import gc, math, logging, fcntl
from time import time, sleep
from struct import calcsize, pack, unpack
from queue import Queue, Empty
from threading import Thread, Event, active_count
from itertools import combinations
from socket import (
    socket, timeout, error, 
    inet_ntoa, inet_aton,
    AF_INET, SOCK_DGRAM, SOCK_STREAM,
    SOL_SOCKET, SO_RCVBUF, SO_SNDBUF,
    )

from numpy import array, nan, fromstring, empty, reshape
from numba import jit
from . import core
from .defines import *
from .xeng import SwarmBaseline, SwarmXengine


SIOCGIFADDR = 0x8915
SIOCSIFHWADDR  = 0x8927
SIOCGIFNETMASK = 0x891b


INNER_RANGE = range(0, SWARM_XENG_PARALLEL_CHAN * 4, 2)
OUTER_RANGE = range(0, SWARM_CHANNELS * 2, SWARM_XENG_TOTAL * 4)
DATA_FID_IND = array(list(j + i for i in OUTER_RANGE for j in INNER_RANGE))


ARRIVAL_THRESHOLD = 0.6
XNUM_TO_LENGTH = SWARM_WALSH_PERIOD / (SWARM_ELEVENTHS * (SWARM_EXT_HB_PER_WCYCLE/SWARM_WALSH_SKIP))


class SwarmDataPackage(object):

    header_prefix_fmt = '<IIIdd'

    def __init__(self, baselines, int_time, int_length):

        # Set all initial members
        self.int_time = int_time
        self.int_length = int_length
        self.baselines = baselines
        self.baselines_i = dict((b, i) for i, b in enumerate(self.baselines))

    @classmethod
    def from_swarm(cls, swarm, int_time=0.0, int_length=0.0):

        # Generate baselines list
        inputs = list(quad[i][j] for quad in swarm for i in range(quad.fids_expected) for j in SWARM_MAPPING_INPUTS)
        cross = list(SwarmBaseline(i, j) for i, j in combinations(inputs, r=2) if SwarmBaseline(i, j).is_valid())
        autos = list(SwarmBaseline(i, i) for i in inputs)
        baselines = autos + cross

        # Create an instance, populate header & data, and return it
        inst = cls(baselines, int_time, int_length)
        inst.init_header()
        inst.init_data()
        return inst

    @classmethod
    def from_string(cls, str_):

        # Generate baselines list
        header_prefix_size = calcsize(cls.header_prefix_fmt)
        n_baselines, n_sidebands, n_channels, int_time, int_length  = unpack(
            cls.header_prefix_fmt, str_[0:header_prefix_size])
        header_size = header_prefix_size + 6 * n_baselines
        baselines_s = reshape(unpack('BBBBBB' * n_baselines, str_[header_prefix_size:header_size]), (n_baselines, 6))
        baselines = list(SwarmBaseline(core.SwarmInput(a, b, c), core.SwarmInput(d, e, f)) for a, b, c, d, e, f in baselines_s)

        # Create an instance, populate header & data, and return it
        data_shape = (n_baselines, n_sidebands, SWARM_CHANNELS*2)
        inst = cls(baselines, int_time, int_length)
        inst.header = str_[:header_size]
        inst.array = fromstring(str_[header_size:], dtype='<f4').reshape(data_shape)
        return inst

    def __getitem__(self, item):
        return self.get(*item)

    def get(self, *item):
        try:
            baseline, sideband = item
            i = self.baselines_i[baseline]
            j = SWARM_XENG_SIDEBANDS.index(sideband)
            return self.array[i, j]
        except:
            raise KeyError("Please only index data package using [baseline, sideband]!")

    def __str__(self):
        return ''.join([self.header, self.array.tostring()])

    def init_header(self):
        hdr_fmt = self.header_prefix_fmt + 'BBBBBB' * len(self.baselines)
        self.header = pack(
            hdr_fmt,
            len(self.baselines),
            len(SWARM_XENG_SIDEBANDS),
            SWARM_CHANNELS,
            self.int_time, self.int_length,
            *list(x for z in self.baselines for y in (z.left, z.right) for x in (y.ant, y.chk, y.pol))
            )

    def init_data(self):

        # Initialize our data array
        data_shape = (len(self.baselines), len(SWARM_XENG_SIDEBANDS), SWARM_CHANNELS*2)
        self.array = empty(shape=data_shape, dtype='<f4')
        self.array[:] = nan

    def set_data(self, xeng_word, fid, data):

        # Get info from the data word
        imag = xeng_word.imag
        imag_off = int(imag)
        baseline = xeng_word.baseline
        sideband = xeng_word.sideband

        # Fill this baseline
        slice_ = compute_slice(DATA_FID_IND, fid, SWARM_XENG_PARALLEL_CHAN, imag_off)
        self.get(baseline, sideband)[slice_] = data

        # Special case for autos, fill imag with zeros
        if baseline.is_auto():
            self.get(baseline, sideband)[slice_+1] = 0.0


@jit
def compute_slice(DATA_FID_IND, fid, SWARM_XENG_PARALLEL_CHAN, imag_off):
    """
    Moved this calculation into its own function in order to make use of numba's @jit feature.
    We gain a %5 improvement from doing this.
    """
    return DATA_FID_IND + fid * SWARM_XENG_PARALLEL_CHAN * 4 + imag_off


class SwarmDataCallback(object):

    def __init__(self, swarm):
        self.__log_name = "Callback:{0}".format(self.__class__.__name__)
        self.logger = logging.getLogger(self.__log_name)
        self.swarm = swarm

    def __call__(self, data):
        pass


class SwarmListener(object):

    def __init__(self, interface, port=4100):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.interface = interface
        self._set_netinfo(port)

    def _set_netinfo(self, port=4100):
        s = socket(AF_INET, SOCK_DGRAM)
        info_adr = fcntl.ioctl(s.fileno(), SIOCGIFADDR, pack('256s', self.interface))
        info_mac = fcntl.ioctl(s.fileno(), SIOCSIFHWADDR, pack('256s', self.interface))
        info_mask = fcntl.ioctl(s.fileno(), SIOCGIFNETMASK, pack('256s', self.interface))
        self.netmask = unpack(SWARM_REG_FMT, info_mask[20:24])[0]
        self.mac = unpack('>Q', '\x00'*2 + info_mac[18:24])[0]
        self.ip = unpack(SWARM_REG_FMT, info_adr[20:24])[0]
        self.host = inet_ntoa(pack(SWARM_REG_FMT, self.ip))
        self.port = port
        s.close()


def has_none(obj):
    try:
        for sub in obj:
            if not isinstance(sub, basestring):
                if has_none(sub):
                    return True
    except TypeError:
        if obj == None:
            return True
    return False


class SwarmDataCatcher:

    def __init__(self, swarm, host='0.0.0.0', port=4100):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.xengines = list(SwarmXengine(quad) for quad in swarm)
        self.swarm = swarm
        self.rawbacks = []
        self.host = host
        self.port = port

        # Catch thread objects
        self.catch_thread = None
        self.catch_queue = Queue()
        self.catch_stop = Event()

        # Ordering thread objects
        self.order_thread = None
        self.order_queue = Queue()
        self.order_stop = Event()

    def get_queue(self):
        return self.order_queue

    def _create_socket(self):
        udp_sock = socket(AF_INET, SOCK_DGRAM)
        udp_sock.bind((self.host, self.port))
        udp_sock.settimeout(2.0)
        return udp_sock

    def start_catch(self):
        if not self.catch_thread:
            self.catch_stop.clear()
            self.catch_thread = Thread(target=self.catch, 
                                       args=(self.catch_stop,
                                             None,
                                             self.catch_queue))
            self.catch_thread.start()
            self.logger.info('Catching thread has started')
        else:
            self.logger.error('Catch thread has not been stopped!')

    def start_order(self):
        if not self.order_thread:
            self.order_stop.clear()
            self.order_thread = Thread(target=self.order, 
                                       args=(self.order_stop, 
                                             self.catch_queue,
                                             self.order_queue))
            self.order_thread.start()
            self.logger.info('Ordering thread has started')
        else:
            self.logger.error('Order thread has not been stopped!')

    def start(self):
        self.start_catch()
        self.start_order()

    def stop_catch(self):
        if self.catch_thread:
            self.catch_stop.set()
            self.catch_thread.join()
            self.catch_thread = None
            self.logger.info('Catch thread has stopped')
        else:
            self.logger.error('Catch thread has not been started!')

    def stop_order(self):
        if self.order_thread:
            self.order_stop.set()
            self.order_thread.join()
            self.order_thread = None
            self.logger.info('Order thread has stopped')
        else:
            self.logger.error('Order thread has not been started!')

    def stop(self):
        self.stop_catch()
        self.stop_order()

    def catch(self, stop, in_queue, out_queue):

        data = {}
        mask = {}
        meta = {}
        udp_sock = self._create_socket()
        while not stop.is_set():

            # Receive a packet and get host info
            try:
                datar, addr = udp_sock.recvfrom(SWARM_VISIBS_PKT_SIZE)
                pkt_time = time() # packet arrival time
            except timeout:
                continue

            # Check if packet is wrong size
            if len(datar) <> SWARM_VISIBS_PKT_SIZE:
                self.logger.warning("Received packet %d:#%d is of wrong size, %d bytes" %(acc_n, pkt_n, len(datar)))
		continue

            # Parse the IP address
            ip = unpack('BBBB', inet_aton(addr[0]))

            # Determing the QID
            qid = (ip[3]>>4) & 0x7

            # Determine the FID
            fid = ip[3] & 0x7

	    # Unpack it to get packet #, accum #, and scan length
            pkt_n, acc_n_mb, acc_n_lh, xnum_mb, xnum_lh = unpack(SWARM_VISIBS_HEADER_FMT, datar[:SWARM_VISIBS_HEADER_SIZE])
            xnum = ((xnum_mb << 16) | xnum_lh) << 5
            acc_n = (acc_n_mb << 16) | acc_n_lh

	    # Initialize qid data buffer, if necessary
            if not data.has_key(qid):
                data[qid] = {}
                mask[qid] = {}
                meta[qid] = {}

	    # Initialize fid data buffer, if necessary
            if not data[qid].has_key(fid):
                data[qid][fid] = {}
                mask[qid][fid] = {}
                meta[qid][fid] = {}

	    # First packet of new accumulation, initalize data buffers
            if not data[qid][fid].has_key(acc_n):
                data[qid][fid][acc_n] = list(None for y in range(SWARM_VISIBS_N_PKTS))
                mask[qid][fid][acc_n] = long(0)
                meta[qid][fid][acc_n] = {
                    'time': pkt_time, # these values correspond to those
                    'xnum': xnum,     # of the first packet of this acc
                    }

            # Check that xnum matches that of first packet
            acc_xnum = meta[qid][fid][acc_n]['xnum']
            if xnum != acc_xnum: # if not, trigger a shutdown and exit
                err_msg = "Received packet %d:#%d has non-matching xnum=%d, should be %d!" %(acc_n, pkt_n, xnum, acc_xnum)
                exception = ValueError(err_msg)
                self.logger.error(err_msg)
                out_queue.put(exception)
                continue

            # Then store data in it
            data[qid][fid][acc_n][pkt_n] = datar[SWARM_VISIBS_HEADER_SIZE:]
            mask[qid][fid][acc_n] |= (1 << pkt_n)

            # If we've gotten all pkts for this acc_n from this FID
            if mask[qid][fid][acc_n] == SWARM_VISIBS_TOTAL:

                # Put data onto the queue
                mask[qid][fid].pop(acc_n)
                datas = ''.join(data[qid][fid].pop(acc_n))
                out_queue.put((qid, fid, acc_n, meta[qid][fid].pop(acc_n), datas))

        udp_sock.close()

    def add_rawback(self, callback, *args, **kwargs):
        inst = callback(self.swarm, *args, **kwargs)
        self.rawbacks.append(inst)

    def _reorder_data(self, datas_list, int_time, int_length):

        # Create data package to hold baseline data
        data_pkg = SwarmDataPackage.from_swarm(self.swarm, int_time=int_time, int_length=int_length)

        for qid, quad in enumerate(self.swarm.quads):

            # Get the xengine packet ordering
            order = list(self.xengines[qid].packet_order())

            # Unpack and reorder each FID's data
            for fid, datas in enumerate(datas_list[qid]):

                # Unpack this FID's data
                data = fromstring(datas, dtype='>i4')

                # Reorder by Xengine word (per channel)
                for offset, word in enumerate(order):

                    if word.baseline.is_valid():
                        sub_data = data[offset::len(order)]
                        data_pkg.set_data(word, fid, sub_data)

        # Return the (hopefully) complete data packages
        return data_pkg

    def order(self, stop, in_queue, out_queue):

        last_acc = []
        primordial = True
        current_acc = None
        for quad in self.swarm.quads:
            last_acc.append(list(None for fid in range(quad.fids_expected)))

        while not stop.is_set():

            # Receive a set of data
            try:
                message = in_queue.get_nowait()
            except Empty:
                sleep(0.01)
                continue

            # Check if we received an exception
            if isinstance(message, Exception):
                out_queue.put(message)
                continue # pass on exemption and move on

            # Otherwise, continue and parse message
            qid, fid, acc_n, meta, datas = message
            this_length = meta['xnum'] * XNUM_TO_LENGTH
            this_time = meta['time']

            # Check if we've started a new scan
            if current_acc is None:

                # Check that first data starts with 0, 0
                if not ((qid == 0) and (fid ==0)):

                    # But, skip if it's a partial primordial scan
                    if primordial:
                        continue
                    else: # error out otherwise
                        err_msg = "Accumulation #{0} started with qid={1}, fid={2}! Should start with 0, 0!".format(acc_n, qid, fid)
                        exception = ValueError(err_msg)
                        self.logger.error(err_msg)
                        out_queue.put(exception)
                        continue

                else: # we're good and can proceed with new scan
                    self.logger.info("First data of accumulation #{0} received".format(acc_n))
                    current_acc = acc_n
                    primordial = False

                    # Initiate the data buffer
                    data = list(list(None for fid in range(quad.fids_expected)) for quad in self.swarm.quads)

                    # Establish meta data for new scan
                    int_length = this_length
                    int_time = this_time

            elif current_acc != acc_n: # not done with scan but scan #'s don't match
                err_msg = "Haven't finished acc. #{0} but received data for acc #{1} from qid={2}, fid={3}".format(current_acc, acc_n, qid, fid)
                exception = ValueError(err_msg)
                self.logger.error(err_msg)
                out_queue.put(exception)
                continue

            # Make sure that all scan lengths match
            if this_length != int_length:
                err_msg = "Data from qid #{0}, fid #{1} has mis-matching scan length: {2:.2f}!".format(qid, fid, this_length)
                exception = ValueError(err_msg)
                self.logger.error(err_msg)
                out_queue.put(exception)
                continue

            # Make sure data arrives within a reasonable time since the first data
            # NOTE: this alone does not enforce any order
            if (this_time - int_time) > ARRIVAL_THRESHOLD:
                err_msg = "Arrival time of qid={0}, fid={1} is too late (>{2:.1f} s from first data)".format(qid, fid, ARRIVAL_THRESHOLD)
                exception = ValueError(err_msg)
                self.logger.error(err_msg)
                out_queue.put(exception)
                continue

            # Populate this data
            try:
                data[qid][fid] = datas
            except IndexError:
                self.logger.info("Ignoring data from unexpected quadrant (qid #{}) or F-engine (fid #{})".format(qid, fid))
                continue # ignore and move on

            # Get the member/fid this set is from
            member = self.swarm[qid][fid]

            # Log the fact
            suffix = "({:.4f} secs since last)".format(time() - last_acc[qid][fid]) if last_acc[qid][fid] else ""
            self.logger.debug("Received full accumulation #{:<4} from qid #{}: {} {}".format(acc_n, qid, member, suffix))

            # Set the last acc time
            last_acc[qid][fid] = time()

	    # If we've gotten all pkts for this acc_n from all QIDs & FIDs
            if not has_none(data):

                # We have all data for this accumulation, log it
                self.logger.info("Received full accumulation #{:<4} with scan length {:.2f} s".format(acc_n, int_length))

                # Do user rawbacks first
                for rawback in self.rawbacks:

                    try: # catch callback error
                        rawback(data)
                    except Exception as exception: # and log if needed
                        self.logger.error("Exception from rawback: {}".format(rawback))
                        out_queue.put(exception)
                        continue

                # Log that we're done with rawbacks
                self.logger.info("Processed all rawbacks for accumulation #{:<4}".format(acc_n))

                # Reorder the xengine data
                data_pkg = self._reorder_data(data, int_time, int_length)
                self.logger.info("Reordered accumulation #{:<4}".format(acc_n))

		# Put data onto queue
                out_queue.put((acc_n, int_time, data_pkg))

                # Done with this accumulation
                current_acc = None


class SwarmDataHandler:

    def __init__(self, swarm, queue):

        # Create initial member variables
        self.logger = logging.getLogger('SwarmDataHandler')
        self.callbacks = []
        self.swarm = swarm
        self.queue = queue

    def add_callback(self, callback, *args, **kwargs):
        inst = callback(self.swarm, *args, **kwargs)
        self.callbacks.append(inst)

    def loop(self, running):

        # Loop until user quits
        while running.is_set():

            try: # to check for data
                message = self.queue.get_nowait()
            except Empty: # none available
                sleep(0.01)
                continue

            # Check if we received an exception
            if isinstance(message, Exception):
                raise message

            # Otherwise, continue and parse message
            acc_n, int_time, data = message

            # Finally, do user callbacks
            for callback in self.callbacks:
                try: # catch callback error
                    callback(data)
                except: # and log if needed
                    self.logger.error("Exception from callback: {}".format(callback))
                    raise

            # Log that we're done with callbacks
            self.logger.info("Processed all callbacks for accumulation #{:<4}".format(acc_n))

            gc.collect() # Force garbage collection
            self.logger.info("Garbage collected. Processing took {:.4f} secs".format(time() - int_time))

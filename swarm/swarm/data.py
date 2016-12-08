import math, logging, fcntl
from time import time, sleep
from struct import pack, unpack
from Queue import Queue, Empty
from threading import Thread, Event, active_count
from itertools import combinations
from socket import (
    socket, timeout, error, 
    inet_ntoa, inet_aton,
    AF_INET, SOCK_DGRAM, SOCK_STREAM,
    SOL_SOCKET, SO_RCVBUF, SO_SNDBUF,
    )

from numpy import array, nan, fromstring, empty

from defines import *
from xeng import (
    SwarmBaseline, 
    SwarmXengine,
    )


SIOCGIFADDR = 0x8915
SIOCSIFHWADDR  = 0x8927
SIOCGIFNETMASK = 0x891b


INNER_RANGE = range(0, SWARM_XENG_PARALLEL_CHAN * 4, 2)
OUTER_RANGE = range(0, SWARM_CHANNELS * 2, SWARM_XENG_TOTAL * 4)
DATA_FID_IND = array(list(j + i for i in OUTER_RANGE for j in INNER_RANGE))

class SwarmDataPackage:

    def __init__(self, swarm, int_time=0.0, length=0.0):

        # Set all initial members
        self.int_time = int_time
        self.int_length = length
        self.inputs = list(quad[i][j] for quad in swarm for i in range(quad.fids_expected) for j in SWARM_MAPPING_INPUTS)
        self._cross = list(SwarmBaseline(i, j) for i, j in combinations(self.inputs, r=2) if SwarmBaseline(i, j).is_valid())
        self._autos = list(SwarmBaseline(i, i) for i in self.inputs)
        self.baselines = self._autos + self._cross
        self.baselines_i = dict((b, i) for i, b in enumerate(self.baselines))
        self._init_data()

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

    def _init_data(self):

        # Initialize our data array
        data_shape = (len(self.baselines), len(SWARM_XENG_SIDEBANDS), SWARM_CHANNELS*2)
        self.array = empty(shape=data_shape, dtype='float32')
        self.array[:] = nan

    def set_data(self, xeng_word, fid, data):

        # Get info from the data word
        imag = xeng_word.imag
        imag_off = int(imag)
        baseline = xeng_word.baseline
        sideband = xeng_word.sideband

        slice_ = DATA_FID_IND + fid * SWARM_XENG_PARALLEL_CHAN * 4 + imag_off
        try: # normal conjugation first

            # Fill this baseline
            self.get(baseline, sideband)[slice_] = data

            # Special case for autos, fill imag with zeros
            if baseline.is_auto():
                self.get(baseline, sideband)[slice_+1] = 0.0

        except KeyError:

            # Conjugate the baseline
            conj_baseline = SwarmBaseline(baseline.right, baseline.left)

            # Try the conjugated baseline
            self.get(conj_baseline, sideband)[slice_] = data


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

        # Interthread signals
        self.new_acc = Event()
        self.new_acc.set()

    def get_queue(self):
        return self.order_queue

    def _create_socket(self):
        udp_sock = socket(AF_INET, SOCK_DGRAM)
        udp_sock.bind((self.host, self.port))
        udp_sock.settimeout(2.0)
        return udp_sock

    def start_catch(self):
        if not self.catch_thread:
            self.catch_thread = Thread(target=self.catch, 
                                       args=(self.catch_stop,
                                             self.new_acc, 
                                             None,
                                             self.catch_queue))
            self.catch_thread.start()
            self.logger.info('Catching thread has started')
        else:
            self.logger.error('Catch thread has not been started!')

    def start_order(self):
        if not self.order_thread:
            self.order_thread = Thread(target=self.order, 
                                       args=(self.order_stop, 
                                             self.new_acc, 
                                             self.catch_queue,
                                             self.order_queue))
            self.order_thread.start()
            self.logger.info('Ordering thread has started')
        else:
            self.logger.error('Order thread has not been started!')

    def start(self):
        self.start_catch()
        self.start_order()

    def stop_catch(self):
        if self.catch_thread:
            self.catch_stop.set()
            self.catch_thread.join()
            self.logger.info('Catch thread has stopped')
        else:
            self.logger.error('Catch thread has not been started!')

    def stop_order(self):
        if self.order_thread:
            self.order_stop.set()
            self.order_thread.join()
            self.logger.info('Order thread has stopped')
        else:
            self.logger.error('Order thread has not been started!')

    def stop(self):
        self.stop_catch()
        self.stop_order()

    def catch(self, stop, new_acc, in_queue, out_queue):

        data = {}
        mask = {}
        udp_sock = self._create_socket()
        while not stop.is_set():

            # Receive a packet and get host info
            try:
                datar, addr = udp_sock.recvfrom(SWARM_VISIBS_PKT_SIZE)
            except timeout:
                continue

            # Send sync if this is the first packet of the next accumulation
            if new_acc.is_set():
                self.logger.info("First packet of new accumulation received")
                int_time = time() # scan end time
                new_acc.clear()

            # Parse the IP address
            ip = unpack('BBBB', inet_aton(addr[0]))

            # Determing the QID
            qid = (ip[3]>>4) & 0x7

            # Determine the FID
            fid = ip[3] & 0x7

	    # Unpack it to get packet # and accum #
            pkt_n, acc_n = unpack(SWARM_VISIBS_HEADER_FMT, datar[:SWARM_VISIBS_HEADER_SIZE])

            # Check if packet is wrong size
            if len(datar) <> SWARM_VISIBS_PKT_SIZE:
                self.logger.error("Received packet %d:#%d is of wrong size, %d bytes" %(acc_n, pkt_n, len(datar)))
		continue

	    # Initialize qid data buffer, if necessary
            if not data.has_key(qid):
                data[qid] = {}
                mask[qid] = {}

	    # Initialize fid data buffer, if necessary
            if not data[qid].has_key(fid):
                data[qid][fid] = {}
                mask[qid][fid] = {}

	    # Initialize acc_n data buffer, if necessary
            if not data[qid][fid].has_key(acc_n):
                data[qid][fid][acc_n] = list(None for y in range(SWARM_VISIBS_N_PKTS))
                mask[qid][fid][acc_n] = long(0)

            # Then store data in it
            data[qid][fid][acc_n][pkt_n] = datar[8:]
            mask[qid][fid][acc_n] |= (1 << pkt_n)

            # If we've gotten all pkts for this acc_n from this FID
            if mask[qid][fid][acc_n] == 2**SWARM_VISIBS_N_PKTS-1:

                # Put data onto the queue
                datas = ''.join(data[qid][fid].pop(acc_n))
                out_queue.put((qid, fid, acc_n, int_time, datas))
                mask[qid][fid].pop(acc_n)

        udp_sock.close()

    def add_rawback(self, callback, *args, **kwargs):
        inst = callback(self.swarm, *args, **kwargs)
        self.rawbacks.append(inst)

    def _reorder_data(self, datas_list, int_time, int_length):

        # Create data package to hold baseline data
        data_pkg = SwarmDataPackage(self.swarm, int_time=int_time, length=int_length)

        for qid, quad in enumerate(self.swarm.quads):

            # Get the xengine packet ordering
            order = list(self.xengines[qid].packet_order())

            # Unpack and reorder each FID's data
            for fid, datas in enumerate(datas_list[qid]):

                # Unpack this FID's data
                data = fromstring(datas, dtype='>i4')

                # We must convert to float efficiently
                data_f = data.view('float32')
                data_f[:] = data

                # Reorder by Xengine word (per channel)
                for offset, word in enumerate(order):

                    if word.baseline.is_valid():
                        sub_data = data[offset::len(order)]
                        data_pkg.set_data(word, fid, sub_data)

        # Return the (hopefully) complete data packages
        return data_pkg

    def order(self, stop, new_acc, in_queue, out_queue):

        data = {}
        last_acc = []
        for quad in self.swarm.quads:
            last_acc.append(list(None for fid in range(quad.fids_expected)))

        while not stop.is_set():

            # Receive a set of data
            try:
                qid, fid, acc_n, int_time, datas = in_queue.get_nowait()
            except Empty:
                sleep(0.01)
                continue

            # Initialize acc_n data buffer, if necessary
            if not data.has_key(acc_n):
                data[acc_n] = []
                for quad in self.swarm.quads:
                    data[acc_n].append(list(None for fid in range(quad.fids_expected)))

            # Populate this data
            try:
                data[acc_n][qid][fid] = datas
            except IndexError:
                self.logger.info("Ignoring data from unexpected quadrant (qid #{}) or F-engine (fid #{})".format(qid, fid))
                continue # ignore and move on

            # Get the member/fid this set is from
            member = self.swarm[qid][fid]

            # Log the fact
            suffix = "({:.4f} secs since last)".format(time() - last_acc[qid][fid]) if last_acc[qid][fid] else ""
            self.logger.info("Received full accumulation #{:<4} from qid #{}, fid #{}: {} {}".format(acc_n, qid, fid, member, suffix))

            # Set the last acc time
            last_acc[qid][fid] = time()

	    # If we've gotten all pkts for this acc_n from all QIDs & FIDs
            if not has_none(data[acc_n]):

                # Flag this accumulation as done
                new_acc.set()

                # We have all data for this accumulation, log it
                self.logger.info("Received full accumulation #{:<4}".format(acc_n))

                # Do user rawbacks first
                for rawback in self.rawbacks:

                    try: # catch callback error
                        rawback(data[acc_n])
                    except KeyboardInterrupt:
                        raise
                    except: # and log if needed
                        self.logger.exception("Exception from rawback: {}".format(rawback))

                # Log that we're done with rawbacks
                self.logger.info("Processed all rawbacks for accumulation #{:<4}".format(acc_n))

                # Reorder the xengine data
                data_pkg = self._reorder_data(data.pop(acc_n), int_time, self.swarm.get_itime())
                self.logger.info("Reordered accumulation #{:<4}".format(acc_n))

		# Put data onto queue
                out_queue.put((acc_n, int_time, data_pkg))


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

    def loop(self):

        try:

            # Loop until user quits
            while True:

                try: # to check for data
                    acc_n, int_time, data = self.queue.get_nowait()
                except Empty: # none available
                    sleep(0.01)
                    continue

                # Finally, do user callbacks
                for callback in self.callbacks:
                    try: # catch callback error
                        callback(data)
                    except KeyboardInterrupt:
                        raise
                    except: # and log if needed
                        self.logger.exception("Exception from callback: {}".format(callback))

                # Log that we're done with callbacks
                self.logger.info("Processed all callbacks for accumulation #{:<4}".format(acc_n))

        except KeyboardInterrupt:

            # User wants to quit
            self.logger.info("Ctrl-C detected. Quitting loop.")

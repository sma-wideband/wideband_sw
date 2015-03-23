import logging, fcntl
from struct import pack, unpack
from socket import (
    socket, inet_ntoa,
    AF_INET, SOCK_DGRAM,
    SO_RCVBUF, SO_SNDBUF,
    timeout,
    )
from Queue import Queue, Empty
from threading import Thread, Event

from numpy import nan, array, empty, uint8, complex128

from core import SwarmROACH
from defines import *


SIOCGIFADDR = 0x8915
SIOCSIFHWADDR  = 0x8927


class Interface(object):

    def __init__(self, mac, ip, port):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.port = port
        self.mac = mac
        self.arp = []
        self.ip = ip


class LocalInterface(Interface):

    def __init__(self, interface, port):
        mac, ip, host = self.get_netinfo(interface)
        super(LocalInterface, self).__init__(mac, ip, port)
        self.interface = interface
        self.host = host

    @staticmethod
    def get_netinfo(interface):
        s = socket(AF_INET, SOCK_DGRAM)
        info_adr = fcntl.ioctl(s.fileno(), SIOCGIFADDR, pack('256s', interface[:15]))
        info_mac = fcntl.ioctl(s.fileno(), SIOCSIFHWADDR, pack('256s', interface[:15]))
        mac = unpack('>Q', '\x00'*2 + info_mac[18:24])[0]
        ip = unpack('>I', info_adr[20:24])[0]
        host = inet_ntoa(pack('>I', ip))
        s.close()
        return mac, ip, host


class DBEImposter(LocalInterface):

    def __init__(self, interface, port=SWARM_BENGINE_PORT):
        super(DBEImposter, self).__init__(interface, port)


class MK6Imposter(LocalInterface):

    def __init__(self, interface, port=SWARM_DBE_PORT):
        super(MK6Imposter, self).__init__(interface, port)


class SwarmDBE(SwarmROACH):

    def __init__(self, swarm, roach2_host):
        super(SwarmROACH, self).__init__(roach2_host)
        self.swarm = swarm

    def setup(self, bitcode, ipbase=0x9d000000, macbase=0x000f9d9d9d00):

        # First program the FPGA
        self._program(bitcode)

        # Configure the RX interfaces
        return self.setup_rx_interfaces(macbase, ipbase)

    def setup_rx_interfaces(self, macbase, ipbase, bhmac=SWARM_BLACK_HOLE_MAC):

        # Keep track of our net info
        self.macbase = macbase
        self.ipbase = ipbase
        self.bhmac = bhmac

        # Initialize the ARP table
        arp = [bh_mac] * 256

        # Determine our TX interfaces first
        fids = range(self.swarm.fids_expected)
        tx_ifaces = list(Interface(macbase + fid, ipbase + fid, SWARM_BENG_PORT) for fid in fids)

        # The determine our RX interfaces
        cores = range(SWARM_DBE_N_RX_CORE)
        rx_ifaces = list(Interface(macbase + 0x1100 + core, ipbase + 0x1100 + core, SWARM_BENG_PORT) for core in cores)

        # Fill the ARP table
        for iface in tx_ifaces + rx_ifaces:
            arp[iface.mac & 0xff] = arp.ip

        # Set ARP table for all interfaces
        for iface in tx_ifaces + rx_ifaces:
            iface.arp = arp

        # Finally configure the interfaces
        for core in cores:

            # Configure each core with net info
            self.roach2.config_10gbe_core(SWARM_DBE_RX_CORE % core, rx_iface[core].mac, rx_iface[core].port, rx_iface[core].arp)

        # Finish up
        return tx_ifaces


class BengineDataCatcher(Thread):

    def __init__(self, queue, stopevent,
                 listen_host, listen_port,
                 pkt_size = SWARM_BENGINE_PKT_SIZE):
        self.queue = queue
        self.pkt_size = pkt_size
        self.stopevent = stopevent
        self.listen_on = (listen_host, listen_port)
        self.logger = logging.getLogger(self.__class__.__name__)
        Thread.__init__(self)

    def _create_socket(self):
        self.udp_sock = socket(AF_INET, SOCK_DGRAM)
        self.udp_sock.bind(self.listen_on)
        self.udp_sock.settimeout(2.0)

    def _close_socket(self):
        self.udp_sock.close()

    def _unpack_pkt(self, datar):

	    # Unpack it to get bcount, fid, and chan id
            bcount_msb, bcount_lsb, fid, chan_id = unpack(SWARM_BENGINE_HEADER_FMT, datar[:SWARM_BENGINE_HEADER_SIZE])
            self.logger.debug("Received chan. id #%d for bcount=#%d from FID=%d" %(chan_id, bcount_lsb, fid))

            # Grab the payload
            payload_bin = datar[SWARM_BENGINE_HEADER_SIZE:]

            # Get the real "bcount"
            bcount = (bcount_msb << 32) | (bcount_lsb)

            return bcount, fid, chan_id, payload_bin

    def run(self):
        self.logger.info('Catching loop has started')
        self._create_socket()

        while not self.stopevent.isSet():

            # Receive a packet and get host info
            try:
                datar, addr = self.udp_sock.recvfrom(self.pkt_size)
            except timeout:
                continue

            # Check if packet is wrong size
            if len(datar) <> self.pkt_size:
                self.logger.error("Received packet is of wrong size, %d bytes" %(len(datar)))
		continue

            # Unpack and put header onto queue
            self.queue.put(self._unpack_pkt(datar))

        self.logger.info('Catching loop has stopped')
        self._close_socket()


class DBEDataCatcher(BengineDataCatcher):

    def _unpack_pkt(self, datar):

        # Coming out of the DBE we have VDIF packets, 
        # so we need to offset into the VDIF packet to get 
        # the underlying Bengine packet
        beng_hdr_offset = 24 # bytes
        beng_data_offset = 40 # bytes

        # Unpack it to get bcount, fid, and chan id
        beng_hdr_bin = datar[beng_hdr_offset:beng_hdr_offset + SWARM_BENGINE_HEADER_SIZE]
        bcount_msb, chan_id, fid, bcount_lsb = unpack('<IHBB', beng_hdr_bin)
        self.logger.debug("Received chan. id #%d for bcount=#%d from FID=%d" %(chan_id, bcount_lsb, fid))

        # Grab the payload
        payload_bin = datar[beng_data_offset:]

        # Get the real "bcount"
        bcount = (bcount_msb << 8) | (bcount_lsb)

        return bcount, fid, chan_id, payload_bin


def convert_to_signed_complex_4bit(in_unsigned_8bit):
    unsigned_real = (in_unsigned_8bit >> 4) & 0xf
    unsigned_imag = (in_unsigned_8bit >> 0) & 0xf
    real = array([r if r<8 else r-16 for r in unsigned_real])
    imag = array([i if i<8 else i-16 for i in unsigned_imag])
    return real + 1j * imag


def convert_to_signed_complex_2bit(in_unsigned_8bit):
    trans_2bit = [-3., -1., 1., 3.]
    unsigned_real = (in_unsigned_8bit >> 4) & 0x3
    unsigned_imag = (in_unsigned_8bit >> 0) & 0x3
    real = array([trans_2bit[r] for r in unsigned_real])
    imag = array([trans_2bit[i] for i in unsigned_imag])
    return real + 1j * imag


class BengineDataHandler:

    def __init__(self, swarm, listener, 
                 catcher=BengineDataCatcher,
                 pkt_size = SWARM_BENGINE_PKT_SIZE,
                 data_converter = convert_to_signed_complex_4bit):

        # Create initial member variables
        self.logger = logging.getLogger(self.__class__.__name__)
        self.data_converter = data_converter
        self.pkt_size = pkt_size
        self.listener = listener
        self.catcher = catcher
        self.swarm = swarm

        # Print out receive side (i.e. listener) network information
        self.logger.info('Listening for data on %s:%d' % (listener.host, listener.port))

    def _unpack_payload(self, payload_bin):
        payload = array(unpack('>%dB' % SWARM_BENGINE_PAYLOAD_SIZE, payload_bin))
        return payload

    def grab_segment(self):

        # Initialize the data array
        data = empty([
                SWARM_N_INPUTS,
                SWARM_TRANSPOSE_SIZE,
                SWARM_CHANNELS,
                ], dtype=complex128)
        data[:] = 1j*nan

        # Initialize tracking variables
        valid_pkts = 0
        target_bcount = None
        segment_recvd = False
        needed_pkts = (SWARM_CHANNELS / (SWARM_N_FIDS / self.swarm.fids_expected)) / SWARM_XENG_PARALLEL_CHAN
        stopper = Event()
        queue = Queue()

        # Create the listening queue
        catcher = self.catcher(queue, stopper, 
            self.listener.host, self.listener.port,
            self.pkt_size)
        catcher.setDaemon(True)

        # Start the data catching loop
        catcher.start()

        # Loop until we have requested segments
        while not segment_recvd:

            try: # to check for data
                bcount, fid, chan_id, payload_bin = queue.get_nowait()
            except Empty: # none available
                continue

            if target_bcount is None: # then this is our first packet

                target_bcount = bcount + 1
                self.logger.info("Target bcount is #{0}".format(target_bcount))

            elif bcount == target_bcount: # this is our target segment!

                # Unpack the payload
                payload = self._unpack_payload(payload_bin)

                # Fill the data array using array slicing
                start_chan = SWARM_XENG_PARALLEL_CHAN * (chan_id * SWARM_N_FIDS + fid)
                stop_chan  = start_chan + SWARM_XENG_PARALLEL_CHAN
                for i_input in range(SWARM_N_INPUTS):
                    for j_spectrum in range(SWARM_TRANSPOSE_SIZE):
                        start_pkt = i_input + j_spectrum * SWARM_N_INPUTS * SWARM_XENG_PARALLEL_CHAN
                        stop_pkt  = start_pkt + SWARM_XENG_PARALLEL_CHAN * SWARM_N_INPUTS
                        partial = self.data_converter(payload[start_pkt:stop_pkt:SWARM_N_INPUTS])
                        data[i_input][j_spectrum][start_chan:stop_chan] = partial

                # Increment the valid packet counter
                valid_pkts += 1

            # If we have all packets we need exit
            if valid_pkts >= needed_pkts:
                segment_recvd = True

        stopper.set()
        return data


class DBEDataHandler(BengineDataHandler):

    def _reorder_payload(self, payload):
        return (payload.reshape(payload.size/4, 4)[:, ::-1]).flatten()

    def _unpack_payload(self, payload_bin):
        payload = self._reorder_payload(array(unpack('%dB' % (SWARM_BENGINE_PAYLOAD_SIZE/2), payload_bin)))
        fluffed = empty(payload.size * 2, dtype=uint8)
        samp_3 = (payload >> 0) & 0x3
        samp_2 = (payload >> 2) & 0x3
        samp_1 = (payload >> 4) & 0x3
        samp_0 = (payload >> 6) & 0x3
        fluffed[0::2] = (samp_0 << 4) + samp_1
        fluffed[1::2] = (samp_2 << 4) + samp_3
        return fluffed

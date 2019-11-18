import logging, os, sys
from time import sleep
from struct import pack, unpack
from random import randint
from socket import inet_ntoa
from threading import Thread
from queue import Queue, Empty
from traceback import format_exception
from collections import OrderedDict

from numpy import array, clip, roll

import casperfpga

from adc5g import (
    pretty_glitch_profile,
    calibrate_mmcm_phase,
    unset_test_mode,
    set_test_mode,
    sync_adc,
    )

import pydsm

from .defines import *
from . import base, xeng, data, qdr, dbe


class ExceptingThread(Thread):

    def __init__(self, queue, logger=None, *args, **kwargs):
        super(ExceptingThread, self).__init__(*args, **kwargs)
        self.logger = logger
        self.queue = queue

    def run(self):
        try:
            super(ExceptingThread, self).run()
        except:
            exc = sys.exc_info()
            if self.logger:
                fmt_exc = format_exception(*exc)
                exc_str = ''.join(fmt_exc[-1])
                for line in exc_str.splitlines():
                    self.logger.error('<{0}> {1}'.format(self.name, line))
            self.queue.put((self, exc))


module_logger = logging.getLogger(__name__)


class SwarmInput:

    def __init__(self, antenna=None, chunk=None, polarization=None, parent_logger=module_logger):

        # Set all initial members
        self.ant = antenna
        self.chk = chunk
        self.pol = polarization

    def __repr__(self):
        repr_str = '{name}(antenna={ant!r}, chunk={chk!r}, polarization={pol!r})'
        return repr_str.format(name=self.__class__.__name__, ant=self.ant, chk=self.chk, pol=self.pol)

    def __str__(self):
        repr_str = 'ant{ant!r}:chk{chk!r}:pol{pol!r}'
        return repr_str.format(ant=self.ant, chk=self.chk, pol=self.pol)

    def __hash__(self):
        return hash((self.ant, self.chk, self.pol))

    def __eq__(self, other):
        if other is not None:
            return (self.ant, self.chk, self.pol) == (other.ant, other.chk, other.pol)
        else:
            return not self.is_valid()

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_valid(self):
        return (self.ant != None) and (self.chk != None) and (self.pol != None)


class SwarmMember(base.SwarmROACH):

    def __init__(self, fid, roach2_host, bitcode=None, dc_if_freqs=[0.0, 0.0], soft_qdr_cal=True,
                 parent_logger=module_logger):
        super(SwarmMember, self).__init__(roach2_host)
        if self.roach2_host:
            self.qdrs = [qdr.SwarmQDR(self.roach2,'qdr%d' % qnum) for qnum in SWARM_ALL_QDR]
        self._inputs = [SwarmInput(parent_logger=self.logger),] * len(SWARM_MAPPING_INPUTS)
        assert len(dc_if_freqs) == 2, \
            "DC IF Frequencies given are invalid: %r" % dc_if_freqs
        self.dc_if_freqs = list(float(i) for i in dc_if_freqs)
        self.soft_qdr_cal = bool(soft_qdr_cal)
        self.bitcode = bitcode
        self.fid = fid
        self.logger = parent_logger.getChild(
            '{name}[fid={fid!r}]'.format(
                name=self.__class__.__name__,
                fid=self.fid,
                )
            )

    def __repr__(self):
        repr_str = '{name}(fid={fid!r}, roach2_host={host!r})[{inputs[0]!r}][{inputs[1]!r}]'
        return repr_str.format(name=self.__class__.__name__, fid=self.fid, host=self.roach2_host, inputs=self._inputs)

    def __str__(self):
        repr_str = '[fid={fid!r}] {host} [{inputs[0]!s}] [{inputs[1]!s}]'
        return repr_str.format(fid=self.fid, host=self.roach2_host, inputs=self._inputs)

    def __getitem__(self, input_n):
        return self._inputs[input_n]

    def get_input(self, input_n):
        return self._inputs[input_n]

    def set_input(self, input_n, input_inst):
        self._inputs[input_n] = input_inst

    def setup(self, qid, fid, fids_expected, itime_sec, noise=randint(0, 15), raise_qdr_err=True):

        # Write to log to show we're starting the setup of this member
        self.logger.info('Configuring ROACH2={host} for transmission as FID #{fid}'.format(host=self.roach2.host, fid=fid))

        # Program the board
        self._program(self.bitcode)

        # Set noise to perfect correlation
        self.set_noise(0xffffffff, 0xffffffff)
        self.reset_digital_noise()

        # ...but actually use the ADCs
        self.set_source(2, 2)

        # Setup our scopes to capture raw data
        self.set_scope(3, 0, 6)

        # Calibrate the ADC MMCM phases
        self.calibrate_adc()

        # If software QDR cal., calibrate and verify QDRs
        if self.soft_qdr_cal:
            self.calibrate_and_verify_qdr(fail_hard=raise_qdr_err)

        # Setup the F-engine
        self._setup_fengine()

        # Setup flat complex gains
        self.set_flat_cgains(0, 2048)
        self.set_flat_cgains(1, 2048)

        # Setup the X-engine
        self._setup_xeng_tvg()
        self.set_itime(itime_sec)
        self.reset_xeng()

        # Initial setup of the switched corner-turn
        self._setup_corner_turn(qid, fid, fids_expected)

        # If hardware QDR cal. only verify:
        if not self.soft_qdr_cal:
            self.verify_qdr()

    def set_digital_seed(self, source_n, seed):

        # Set the seed for internal noise
        seed_bin = pack(SWARM_REG_FMT, seed)
        self.roach2.write(SWARM_SOURCE_SEED % source_n, seed_bin)

    def set_noise(self, seed_0, seed_1):

        # Setup our digital noise
        self.set_digital_seed(0, seed_0)
        self.set_digital_seed(1, seed_1)
 
    def reset_digital_noise(self, source_0=True, source_1=True):

        # Reset the given sources by twiddling the right bits
        mask = (source_1 << 31) + (source_0 << 30)
        val = self.roach2.read_uint(SWARM_SOURCE_CTRL)
        self.roach2.write(SWARM_SOURCE_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_SOURCE_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_SOURCE_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def set_source(self, source_0, source_1):

        # Set our sources to the given values
        ctrl_bin = pack(SWARM_REG_FMT, (source_1<<3) + source_0)
        self.roach2.write(SWARM_SOURCE_CTRL, ctrl_bin)

    def set_scope(self, sync_out, scope_0, scope_1):

        # Set our scopes to the given values
        ctrl_bin = pack(SWARM_REG_FMT, (sync_out<<16) + (scope_1<<8) + scope_0)
        self.roach2.write(SWARM_SCOPE_CTRL, ctrl_bin)

    def calibrate_adc(self):

        # Set ADCs to test mode
        for inp in SWARM_MAPPING_INPUTS:
            set_test_mode(self.roach2, inp)

        # Send a sync
        sync_adc(self.roach2)

        # Do the calibration
        for inp in SWARM_MAPPING_INPUTS:
            opt, glitches = calibrate_mmcm_phase(self.roach2, inp, [SWARM_SCOPE_SNAP % inp,])
            gprof = pretty_glitch_profile(opt, glitches)
            if opt is None:
                self.logger.error('ADC%d calibration failed! Glitch profile: [%s]' % (inp, gprof))
            else:
                self.logger.info( 'ADC%d calibration found optimal phase: %2d [%s]' % (inp, opt, gprof))

        # Unset test modes
        for inp in SWARM_MAPPING_INPUTS:
            unset_test_mode(self.roach2, inp)

    def warm_calibrate_adc(self):

        # This time do one input at a time
        for inp in SWARM_MAPPING_INPUTS:

            # First set to test mode
            set_test_mode(self.roach2, inp)

            # Send a sync
            if inp != 0: # if not ZDOK0
                sync_adc(self.roach2, zdok_0=False)

            # Do the calibration
            opt, glitches = calibrate_mmcm_phase(self.roach2, inp, [SWARM_SCOPE_SNAP % inp,])
            gprof = pretty_glitch_profile(opt, glitches)
            if opt is None:
                self.logger.error('ADC%d calibration failed! Glitch profile: [%s]' % (inp, gprof))
            else:
                self.logger.info( 'ADC%d calibration found optimal phase: %2d [%s]' % (inp, opt, gprof))

            # Unset test mode
            unset_test_mode(self.roach2, inp)

    def _setup_fengine(self):

        # Set the shift schedule of the F-engine
        sched_bin = pack(SWARM_REG_FMT, SWARM_SHIFT_SCHEDULE)
        self.roach2.write(SWARM_FENGINE_CTRL, sched_bin)

    def set_flat_cgains(self, input_n, flat_value):

        # Set gains for input to a flat value
        gains = [flat_value,] * SWARM_CHANNELS
        gains_bin = pack('>%dH' % SWARM_CHANNELS, *gains)
        self.roach2.write(SWARM_CGAIN_GAIN % input_n, gains_bin)


    def set_bengine_gains(self, psum_gain_0, psum_gain_1):

        # Scale the user value by the fractional bits and concatenate
        psum_reg = (int(round(psum_gain_1 * 16)) << 8) + int(round(psum_gain_0 * 16))

        # Write value to the register
        self.roach2.write_int(SWARM_BENGINE_GAIN, psum_reg)

    def reset_xeng(self):

        # Twiddle bit 29
        mask = 1 << 29 # reset bit location
        val = self.roach2.read_uint(SWARM_XENG_CTRL)
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def get_bengine_mask(self):

        # Get the phased sum mask
        return self.roach2.read_uint(SWARM_BENGINE_DISABLE) ^ 0xffff

    def set_bengine_mask(self, mask):

        # Set the phased sum mask
        disable = (mask & 0xffff) ^ 0xffff
        return self.roach2.write_int(SWARM_BENGINE_DISABLE, disable)

    def get_itime(self):
        
        # Get the current integration time in spectra
        try: # use DSM first
            xeng_time = pydsm.read(self.roach2.host, SWARM_SCAN_DSM_NAME)[SWARM_SCAN_LENGTH][0]
            self.logger.debug("DSM scan length retrieved")
        except: # in case of failure, use katcp
            xeng_time = self.roach2.read_uint(SWARM_XENG_XN_NUM) & 0x1fffffff

        # Convert it to seconds and return
        cycles = xeng_time / (SWARM_ELEVENTHS * (SWARM_EXT_HB_PER_WCYCLE/SWARM_WALSH_SKIP))
        return cycles * SWARM_WALSH_PERIOD

    def set_itime(self, itime_sec):

        # Set the integration (SWARM_ELEVENTHS spectra per step * steps per cycle)
        self._xeng_itime = SWARM_ELEVENTHS * (SWARM_EXT_HB_PER_WCYCLE/SWARM_WALSH_SKIP) * int(itime_sec/SWARM_WALSH_PERIOD)
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, self._xeng_itime & 0x1fffffff))

    def _reset_corner_turn(self):

        # Twiddle bits 31 and 30
        mask = (1 << 31) + (1 << 30)
        val = self.roach2.read_uint(SWARM_NETWORK_CTRL)
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def _setup_corner_turn(self, qid, fid, fids_expected, bh_mac=SWARM_BLACK_HOLE_MAC):

        # Reset the cores
        self._reset_corner_turn()

        # Store our FID 
        self.fid = fid
        self.fids_expected = fids_expected

        # Set up initial parameters
        ipbase = 0xc0a88000 + (qid<<8)
        macbase = 0x000f530cd500 + (qid<<8)

        # Set static parameters
        self.roach2.write_int(SWARM_NETWORK_FIDS_EXPECTED, self.fids_expected)
        self.roach2.write_int(SWARM_NETWORK_IPBASE, ipbase)
        self.roach2.write_int(SWARM_NETWORK_FID, self.fid)

        # Initialize the ARP table 
        arp = [bh_mac] * 256

        # Fill the ARP table
        for fid in range(self.fids_expected):
            for core in SWARM_ALL_CORE:
                last_byte = (fid << 4) + 0b1100 + core
                arp[last_byte] = macbase + last_byte

        # Configure 10 GbE devices
        for core in SWARM_ALL_CORE:
            name = SWARM_NETWORK_CORE % core
            last_byte = (self.fid << 4) + 0b1100 + core
            self.roach2.config_10gbe_core(name, macbase + last_byte, ipbase + last_byte, 18008, arp)

        # Lastly enable the TX only (for now)
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x20))

    def setup_beamformer(self, qid, dest, bh_mac=SWARM_BLACK_HOLE_MAC):

        # Initialize the ARP table 
        arp = [bh_mac] * 256

        # Set up initial parametrs
        ipbase = 0xc0a80b00 + (qid<<8)
        macbase = 0x000f530ce500 + (qid<<8)

        # Set the MAC for our destinatino IP
        arp[dest.ip & 0xff] = dest.mac

        # Configure 10 GbE device
        last_byte = (self.fid << 4) + 0b1100
        self.roach2.config_10gbe_core(SWARM_BENGINE_CORE, macbase + last_byte, ipbase + last_byte, SWARM_BENGINE_PORT, arp)

        # Configure the B-engine destination IPs
        self.roach2.write(SWARM_BENGINE_SENDTO_IP, pack(SWARM_REG_FMT, dest.ip))

        # Reset the 10 GbE cores before enabling
        self.roach2.write(SWARM_BENGINE_CTRL, pack(SWARM_REG_FMT, 0x00000000))
        self.roach2.write(SWARM_BENGINE_CTRL, pack(SWARM_REG_FMT, 0x40000000))
        self.roach2.write(SWARM_BENGINE_CTRL, pack(SWARM_REG_FMT, 0x00000000))

        # Configure the B-engine gains (for optimal state counts)
        self.set_bengine_gains(10.0, 10.0)

        # Lastly enable the TX 
        self.roach2.write(SWARM_BENGINE_CTRL, pack(SWARM_REG_FMT, 0x80000000))

    def reset_ddr3(self):

        # Twiddle bit 30
        mask = 1 << 30 # reset bit location
        val = self.roach2.read_uint(SWARM_VISIBS_DELAY_CTRL)
        self.roach2.write(SWARM_VISIBS_DELAY_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_VISIBS_DELAY_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_VISIBS_DELAY_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def xengine_tvg(self, enable=False):

        # Disable/enable using bit 31
        mask = 1 << 31 # enable bit location
        val = self.roach2.read_uint(SWARM_XENG_CTRL)
        if enable:
            self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val | mask))
        else:
            self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def _setup_xeng_tvg(self):

        # Give each input a different constant value
        const_inputs = [0x0102, 0x0304, 0x0506, 0x0708, 0x090a, 0x0b0c, 0x0d0e, 0x0f10] * (SWARM_VISIBS_CHANNELS/8)
        for i in SWARM_ALL_FID:
            self.roach2.write(SWARM_XENG_TVG % i, pack('>%dH' % SWARM_VISIBS_CHANNELS, *const_inputs))

    def visibs_delay(self, qid, enable=True, delay_test=False):

        # Disable/enable Laura's DDR3 delay and test
        this_delay = SWARM_VISIBS_CHUNK_DELAY * (qid * SWARM_N_FIDS + self.fid)
        self.roach2.write_int(SWARM_VISIBS_DELAY_CTRL, (enable<<31) + (delay_test<<29) + this_delay)

    def qdr_ready(self, qdr_num=0):

        # get the QDR status
        status = self.roach2.read_uint(SWARM_QDR_CTRL % qdr_num, offset=1)
        phy_rdy = bool(status & 1)
        cal_fail = bool((status >> 8) & 1)
        #print 'fid %s qdr%d status %s' %(self.fid, qdr_num, stat)
        return phy_rdy and not cal_fail

    def reset_qdr(self, qdr_num=0):

        # set the QDR status
        self.roach2.blindwrite(SWARM_QDR_CTRL % qdr_num, pack(SWARM_REG_FMT, 0xffffffff))
        self.roach2.blindwrite(SWARM_QDR_CTRL % qdr_num, pack(SWARM_REG_FMT, 0x0))

    def calibrate_and_verify_qdr(self, max_tries=3, fail_hard=True):

        # calibrate each QDR
        for qnum in SWARM_ALL_QDR:
            self.logger.debug('checking QDR memory %d' % qnum)

            try_n = 0
            while not self.qdrs[qnum].qdr_cal(fail_hard=False):

                # try up to max number of tries
                if try_n < max_tries:
                    self.logger.warning('QDR memory {0} not ready, retrying calibration (try #{1})'.format(qnum, try_n))
                    try_n += 1

                # max tries exceded, gtfo
                else:
                    msg = 'QDR memory {0} not calibrating, tried max number of tries'.format(qnum)
                    self.logger.error(msg)
                    if fail_hard:
                        raise RuntimeError(msg)
                    else:
                        break

            if self.qdrs[qnum].qdr_cal_check():
                self.logger.info('QDR memory {0} calibrated successfully'.format(qnum))

    def verify_qdr(self, max_tries=10):
  
        # verify each QDR
        for qnum in SWARM_ALL_QDR:
            self.logger.debug('checking QDR memory %d' % qnum)

            try_n = 0
            while not self.qdr_ready(qnum):

                # reset up to max number of tries
                if try_n < max_tries:
                    self.logger.warning('QDR memory {0} not ready, resetting (try #{1})'.format(qnum, try_n))
                    self.reset_qdr(qnum)
                    try_n += 1

                # max tries exceded, gtfo
                else:
                    msg = 'QDR memory {0} not calibrating, reset max number of times'
                    self.logger.error(msg)
                    raise RuntimeError(msg)

            self.logger.debug('QDR memory {0} verified successfully'.format(qnum))

    def setup_visibs(self, qid, listener, delay_test=False):

        # From the SWARM network specifications
        # MAC = 02:53:57:41:[0x4D + (NID=1)<<4]:[QID<<4 + 0x8 + FID]
        # IP = 53:4D:[0x41 + (NID=1)<<4]:[QID<<4 + 0x8 + FID]
        # PORT = 18008 + [(NID=1)<<4 + QID]

        # Set up initial parameters
        nid = 1 # simply stands for visibility type data
        mac = 0x025357410000 + ((0x4D + (nid<<4))<<8) + ((qid<<4) + 0x8 + self.fid)
        ip = 0x534D0000 + ((0x41 + (nid<<4))<<8) + ((qid<<4) + 0x8 + self.fid)
        port = 18008 + (nid<<4) + qid

        # Log the transmit side net info
        tip_str = inet_ntoa(pack(SWARM_REG_FMT, ip))
        self.logger.info("Transmit interface set to {0}:{1}".format(tip_str, port))

        # Make sure listener subnet matches
        if not (ip & 0xFFFFFF00) == (listener.ip & listener.netmask):
            vip_str = inet_ntoa(pack(SWARM_REG_FMT, ip))
            lip_str = inet_ntoa(pack(SWARM_REG_FMT, listener.ip))
            raise RuntimeError("Listener IP ({0}) not in same subnet as visibility IP ({1})".format(lip_str, vip_str))

        # Reset the DDR3
        self.reset_ddr3()

        # Enable DDR3 interleaver
        self.visibs_delay(qid, enable=True, delay_test=delay_test)

        # Fill the visibs ARP table
        arp = [SWARM_BLACK_HOLE_MAC] * 256
        arp[listener.ip & 0xff] = listener.mac

        # Configure the transmit interface
        self.roach2.config_10gbe_core(SWARM_VISIBS_CORE, mac, ip, port, arp)

        # Configure the visibility packet buffer
        self.roach2.write(SWARM_VISIBS_SENDTO_IP, pack(SWARM_REG_FMT, listener.ip))
        self.roach2.write(SWARM_VISIBS_SENDTO_PORT, pack(SWARM_REG_FMT, listener.port))

        # Reset (and disable) visibility transmission
        self.roach2.write(SWARM_VISIBS_TENGBE_CTRL, pack(SWARM_REG_FMT, 1<<30))
        self.roach2.write(SWARM_VISIBS_TENGBE_CTRL, pack(SWARM_REG_FMT, 0))

        # Finally enable transmission
        self.roach2.write(SWARM_VISIBS_TENGBE_CTRL, pack(SWARM_REG_FMT, 1<<31))

    def get_visibs_ip(self):

        # Return the visibs core IPs
        return inet_ntoa(self.roach2.read(SWARM_VISIBS_CORE, 4, offset=0x10))

    def sync_sowf(self):

        # Twiddle bit 31
        mask = 1 << 31 # reset bit location
        val = self.roach2.read_uint(SWARM_SYNC_CTRL)
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def sync_1pps(self):

        # Twiddle bit 30
        mask = 1 << 30 # reset bit location
        val = self.roach2.read_uint(SWARM_SYNC_CTRL)
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def sync_mcnt(self):

        # Twiddle bit 29
        mask = 1 << 29 # reset bit location
        val = self.roach2.read_uint(SWARM_SYNC_CTRL)
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def sync_beng(self):

        # Twiddle bit 28
        mask = 1 << 28 # reset bit location
        val = self.roach2.read_uint(SWARM_SYNC_CTRL)
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def sync_rtime(self):

        # Twiddle bit 28
        mask = 1 << 27 # reset bit location
        val = self.roach2.read_uint(SWARM_SYNC_CTRL)
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_SYNC_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def enable_network(self):

        # Enable the RX and TX
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x30))

    def fringe_stop(self, enable):

        # Stop fringe stopping
        try:
            self.send_katcp_cmd(SWARM_FSTOP_STOP_CMD)
        except RuntimeError:
            self.logger.error("Stopping fringe stopping failed!")

        # Start it again (if requested)
        if enable:

            try:
                self.send_katcp_cmd(SWARM_FSTOP_START_CMD)
            except RuntimeError:
                self.logger.error("Starting fringe starting failed!")

            # If we've been given IF frequencies set them
            if all(self.dc_if_freqs):

                try:
                    katcp_cmd_str = SWARM_FSTOP_SET_CMD.format(*self.dc_if_freqs)
                    self.send_katcp_cmd(*katcp_cmd_str.split())
                except RuntimeError:
                    self.logger.error("Starting fringe starting failed!")

    def dewalsh(self, enable_0, enable_1):

        # Set the Walsh control register
        self.roach2.write(SWARM_WALSH_CTRL, pack(SWARM_REG_FMT, (enable_1<<30) + (enable_0<<28) + 0xfffff))

    def set_walsh_pattern(self, input_n, pattern, offset=0):

        # Get the current Walsh table
        walsh_table_bin = self.roach2.read(SWARM_WALSH_TABLE_BRAM, SWARM_WALSH_TABLE_LEN*4)
        walsh_table = list(unpack('>%dI' % SWARM_WALSH_TABLE_LEN, walsh_table_bin))

        # Find out many repeats we need
        pattern_size = len(pattern) / SWARM_WALSH_SKIP
        repeats = SWARM_WALSH_TABLE_LEN / pattern_size

        # Repeat the pattern as needed
        for rep in range(repeats):

            # Go through each step (with skips)
            for step in range(pattern_size):

                # Get the requested Walsh phase
                index = ((step + offset) * SWARM_WALSH_SKIP) % len(pattern)
                phase = int(pattern[index])

                # Swap 90 if requested
                if self.dc_if_freqs[input_n] >= 0.0:
                    if phase == 1:
                        phase = 3
                    elif phase == 3:
                        phase = 1

                # Get the current value in table
                current = walsh_table[rep*pattern_size + step]

                # Mask in our phase
                shift_by = input_n * 4
                mask = 0xf << shift_by
                new = (current & ~mask) | (phase << shift_by)
                walsh_table[rep*pattern_size + step] = new

        # Finally write the updated table back
        walsh_table_bin = pack('>%dI' % SWARM_WALSH_TABLE_LEN, *walsh_table)
        self.roach2.write(SWARM_WALSH_TABLE_BRAM, walsh_table_bin)

    def set_sideband_states(self, sb_states):

        # Write the states to the right BRAM
        sb_states_bin = pack('>%dB' % (len(sb_states)), *sb_states)
        self.roach2.write(SWARM_SB_STATE_BRAM, sb_states_bin)

    def get_delay(self, input_n):

        # Get the DSM response
        try:
            dsm_reply = pydsm.read(self.roach2.host, SWARM_FIXED_OFFSETS_DSM_NAME)
        except:
            self.logger.exception("DSM read failed")
            return None

        # Grab the requested delay
        delay = dsm_reply[SWARM_FIXED_OFFSETS_DELAY][0][input_n]
        return delay

    def set_delay(self, input_n, value):

        # Get the DSM response first
        try:
            dsm_reply = pydsm.read(self.roach2.host, SWARM_FIXED_OFFSETS_DSM_NAME)
        except:
            self.logger.exception("DSM read failed")
            return None

        # Write in our new value
        dsm_reply_back = {
            SWARM_FIXED_OFFSETS_DELAY: list(dsm_reply[SWARM_FIXED_OFFSETS_DELAY][0]),
            SWARM_FIXED_OFFSETS_PHASE: list(dsm_reply[SWARM_FIXED_OFFSETS_PHASE][0]),
            }
        dsm_reply_back[SWARM_FIXED_OFFSETS_DELAY][input_n] = value

        # Send our modified DSM request back
        try:
            pydsm.write(self.roach2.host, SWARM_FIXED_OFFSETS_DSM_NAME, dsm_reply_back)
        except:
            self.logger.exception("DSM read failed")

    def get_phase(self, input_n):

        # Get the DSM response
        try:
            dsm_reply = pydsm.read(self.roach2.host, SWARM_FIXED_OFFSETS_DSM_NAME)
        except:
            self.logger.exception("DSM read failed")
            return None

        # Grab the requested phase
        phase = dsm_reply[SWARM_FIXED_OFFSETS_PHASE][0][input_n]
        return phase

    def set_phase(self, input_n, value):

        # Get the DSM response first
        try:
            dsm_reply = pydsm.read(self.roach2.host, SWARM_FIXED_OFFSETS_DSM_NAME)
        except:
            self.logger.exception("DSM read failed")
            return None

        # Write in our new value
        dsm_reply_back = {
            SWARM_FIXED_OFFSETS_PHASE: list(dsm_reply[SWARM_FIXED_OFFSETS_PHASE][0]),
            SWARM_FIXED_OFFSETS_PHASE: list(dsm_reply[SWARM_FIXED_OFFSETS_PHASE][0]),
            }
        dsm_reply_back[SWARM_FIXED_OFFSETS_PHASE][input_n] = value

        # Send our modified DSM request back
        try:
            pydsm.write(self.roach2.host, SWARM_FIXED_OFFSETS_DSM_NAME, dsm_reply_back)
        except:
            self.logger.exception("DSM read failed")


class SwarmQuadrant:

    def __init__(self, qid, map_filename, walsh_filename=SWARM_WALSH_PATTERNS, parent_logger=module_logger):

        # Set initial member variables
        self.qid = qid
        self.logger = parent_logger.getChild(
            '{name}[qid={qid!r}]'.format(
                name=self.__class__.__name__,
                qid=self.qid,
                )
            )

        # Parse mapping for first time
        self.load_mapping(map_filename)

        # Parse Walsh patterns
        self.load_walsh_patterns(walsh_filename)

        # Using patterns derive sideband states
        self.load_sideband_states()

    def __len__(self):
        return len(self.members)

    def __getitem__(self, fid):
        return self.members.values()[fid]

    def __repr__(self):
        return '{name}(qid={qid}, members=[{members}])'.format(name=self.__class__.__name__, qid=self.qid, members=self.members)

    def __str__(self):
        return os.linesep.join('[qid={0!r}] {1:s}'.format(self.qid, m) for f, m in self.get_valid_members())

    def __dir__(self):
        return self.__dict__.keys() + dir(SwarmQuadrant) + dir(SwarmMember) + ['roach2', ]

    def __getattr__(self, attr):

        # See if the non-SwarmQuadrant attribute is a SwarmMember method
        if callable(getattr(self.members.values()[0], attr, None)):

            # If so, return a callable that passes the users args and kwargs
            # to the appropriate method on all members
            self.logger.debug("{0} is not a SwarmQuadrant method but it is a SwarmMember method; "
                             "calling on all members!".format(attr))
            return lambda *args, **kwargs: self.members_do(lambda fid, member: getattr(member, attr)(*args, **kwargs))

        # See if user is trying to access the SwarmMember.roach2 attribute
        if attr == 'roach2':

            # Define an anonymous class we will instantiate and return
            class AnonClass(object):
                def __dir__(self):
                    return dir(FpgaClient)
                def __getattr__(self_, attr):
                    if callable(getattr(FpgaClient, attr, None)):
                        return lambda *args, **kwargs: self.members_do(lambda fid, member: getattr(member.roach2, attr)(*args, **kwargs))
                    else:
                        return self.members_do(lambda fid, member: getattr(member.roach2, attr))

            return AnonClass()

        else: # Remember to raise an error if nothing is found
            raise AttributeError("{0} not an attribute of SwarmQuadrant or SwarmMember".format(attr))

    def load_mapping(self, map_filename):

        # Clear the members instance
        self.members = OrderedDict()

        # Store (or restore) current mapping file
        self.map_filename = map_filename

        # Open the mapping file
        with open(self.map_filename, 'r') as map_file:

            # Initialize FID
            fid = 0

            # Initialize empty parameters
            parameters = {}

            # Parse line by line
            for map_line in map_file:

                # Splits by (and removes) whitespace
                entry = map_line.split()

                # Checks if this line is a comment
                is_comment = entry[0].startswith(SWARM_MAPPING_COMMENT)

                # Checks if our line is a SwarmQuadrant parameter
                is_quadrant_parameter = entry[0].startswith(SWARM_MAPPING_QUAD_PARAM)

                # Checks if our line is a SwarmMember parameter
                is_member_parameter = entry[0].startswith(SWARM_MAPPING_MEM_PARAM)

                if is_comment:

                    # Display map comment
                    self.logger.debug('Mapping comment found: %s' % map_line.rstrip())

                elif is_quadrant_parameter:

                    # Set the attribute but DO NOT override
                    if not hasattr(self, entry[1]):
                        setattr(self, entry[1], True if len(entry[2:]) == 0 else ' '.join(entry[2:]))
                    else:
                        self.logger.warning('Ignoring quadrant parameter {0}; will not override existing attributes'.format(entry[1]))

                    # Display map parameter
                    self.logger.debug('Quadrant attribute set: {0}={1}'.format(entry[1], getattr(self, entry[1])))

                elif is_member_parameter:

                    # Set the parameter
                    parameters[entry[1]] = entry[2] if len(entry[2:]) == 1 else entry[2:]

                    # Display map parameter
                    self.logger.debug('Member parameters updated: {0}'.format(parameters))

                else:

                    # Make sure entry has the right columns
                    n_cols = len(entry)
                    if n_cols < SWARM_MAPPING_COLUMNS:
                        self.logger.error('Not enough columns in map, ignoring line: %s' % map_line)
                        continue
                    elif n_cols > SWARM_MAPPING_COLUMNS:
                        self.logger.error('Too many columns in map, ignoring line: %s' % map_line)
                        continue
                    
                    # First column is ROACH2 number
                    roach2_num = int(entry[0], base=16)
                    roach2_host = SWARM_ROACH2_IP % roach2_num

                    # Create and attach our member instance
                    member_inst = SwarmMember(fid, roach2_host, parent_logger=self.logger, **parameters)
                    if not self.members.has_key(roach2_host):
                        self.members[roach2_host] = member_inst
                        fid += 1

                    # Second column is input number
                    roach2_inp = int(entry[1])
                    if roach2_inp not in SWARM_MAPPING_INPUTS:
                        self.logger.error('%d not a valid SWARM input!' % roach2_inp)
                        continue

                    # Third column is antenna number
                    antenna = int(entry[2])

                    # Fourth column is chunk number
                    chunk = int(entry[3])
                    if chunk not in SWARM_MAPPING_CHUNKS:
                        self.logger.error('%d not a valid SWARM chunk!' % chunk)
                        continue

                    # Fifth column is polarization
                    pol = int(entry[4])
                    if pol not in SWARM_MAPPING_POLS:
                        self.logger.error('%d not a valid SWARM polarization!' % pol)
                        continue

                    # Finally, attach our input instance
                    input_inst = SwarmInput(antenna=antenna, chunk=chunk, polarization=pol,
                                            parent_logger=member_inst.logger)
                    self.members[roach2_host].set_input(roach2_inp, input_inst)

                    # We're done, spit some info out
                    self.logger.debug('Mapping antenna=%r to %s:input=%d', 
                                      input_inst, roach2_host, roach2_inp)

        # Set number FIDs expected
        self.fids_expected = fid

        # Fill missing FIDs with empty members
        missing_fids = set(SWARM_ALL_FID) - set(range(self.fids_expected))
        for missing_fid in range(fid, SWARM_N_FIDS):
            self.members[missing_fid] = SwarmMember(missing_fid, None, parent_logger=self.logger)

    def get_valid_members(self):

        # Return a list of valid fids and members
        for fid in range(self.fids_expected):
            yield fid, self[fid]

    def load_walsh_patterns(self, walsh_filename):

        # Clear the Walsh patterns instance
        self.walsh_patterns = OrderedDict()

        # Store (or restore) current Walsh pattern file
        self.walsh_filename = walsh_filename

        # Open the Walsh pattern file
        with open(self.walsh_filename, 'r') as walsh_file:

            # Parse line by line
            for walsh_line in walsh_file:

                # Splits by (and removes) whitespace
                entry = walsh_line.split()

                # Ignore empty lines
                if len(entry) == 0:
                    continue

                # Checks if this line is a comment
                is_comment = entry[0].startswith(SWARM_MAPPING_COMMENT)

                if is_comment:

                    # Display map comment
                    self.logger.debug('Mapping comment found: %s' % walsh_line.rstrip())

                else:

                    # First column is antenna number
                    ant = int(entry[0])

                    # Second column is the pattern
                    pattern = entry[1]

                    # Display Walsh pattern we found
                    self.logger.debug('Length %d Walsh pattern found for antenna #%d' % (len(pattern), ant))

                    # Add it to your patterns
                    self.walsh_patterns[ant] = pattern

    def set_walsh_patterns(self, offset=0):

        # Go through every member
        for fid, member in self.get_valid_members():

            # And every input
            for inp in SWARM_MAPPING_INPUTS:

                # Get the antenna for this input
                ant = member[inp].ant

                # Get the pattern for the antenna
                pattern = self.walsh_patterns[ant]

                # Then set it using the member function
                member.set_walsh_pattern(inp, pattern, offset=offset)

            # Enable de-Walshing
            member.dewalsh(enable_0=3, enable_1=3)

    def load_sideband_states(self):

        # Initialize sideband states
        self.sideband_states = list()

        # Get the Xengine output order
        order = list(xeng.SwarmXengine(self).xengine_order())

        # For each HB in a single SOWF cycle
        for hb in range(SWARM_INT_HB_PER_SOWF):

            # Find out step in the Walsh pattern
            step = (hb * SWARM_WALSH_SKIP) % SWARM_EXT_HB_PER_WCYCLE

            # Go through each Xengine output word
            for i in range(0, len(order), 2):

                # We skip by two since the Xengine
                # outputs real and imaginary at once
                word = order[i]

                # If this is an auto or cross-chunk, 
                # there is no sideband separation
                if word.is_auto() or not word.is_valid():
                    self.sideband_states.append(1)
                else:

                    # Get 90/270 Walsh state for both antennas
                    state_left = int(self.walsh_patterns[word.left.ant][step]) & 1
                    state_right = int(self.walsh_patterns[word.right.ant][step]) & 1

                    # Append the combined state
                    self.sideband_states.append(int(state_left == state_right))

    def set_sideband_states(self):

        # Go through every member
        for fid, member in self.get_valid_members():

            # Write the states to the FPGA
            member.set_sideband_states(self.sideband_states)

    def fringe_stopping(self, enable):

        # Go through every member
        for fid, member in self.get_valid_members():

            # En(dis)able fringe stoppiner per member
            member.fringe_stop(enable)

    def setup_beamformer(self):

        # Don't do anything if this quadrant doesn't have SDBE
        if not hasattr(self, 'sdbe'):
            return
        else: # we have an SBDE continue

            # Create the SDBE interface object and pass it along
            for fid, member in self.get_valid_members():
                core = fid >> 1
                iface = base.Interface(
                    0x000f9d9d9d00 + ((self.qid+1) << 4) + core,    # the SDBE's MAC and IP values
                    0xc0a80000 + ((self.qid+11) << 8) + core + 100, # are hard-coded for now so make sure
                    SWARM_BENGINE_PORT)                        # they match what the SDBE is set up for
                member.setup_beamformer(self.qid, iface)

    def get_beamformer_inputs(self):

        mask = None
        inputs = []

        # Get the phased sum mask from all members
        for fid, member in self.get_valid_members():

            # Set our first mask
            if fid == 0:
                mask = member.get_bengine_mask()
            else:
                if member.get_bengine_mask() != mask:
                    err_msg = 'FID #%d has mismatching phased sum mask!' % fid
                    self.logger.error(err_msg)
                    raise ValueError(err_msg)

        # Now convert our mask to a list of inputs
        for input_n in SWARM_MAPPING_INPUTS:
            for fid in SWARM_ALL_FID:
                if mask & 0x1:
                    inputs.append(self[fid][input_n])
                mask >>= 1

        return inputs

    def set_beamformer_inputs(self, inputs):

        mask = 0x0

        # Convert list of inputs to a mask
        for input_n in SWARM_MAPPING_INPUTS:
            for fid in SWARM_ALL_FID:
                if self[fid][input_n] in inputs:
                    input_mask = 0x1 << (fid + input_n * SWARM_N_FIDS)
                    mask |= input_mask

        # Set the mask on all members
        for fid, member in self.get_valid_members():
            member.set_bengine_mask(mask)

    def get_itime(self):

        itime = None

        # Go through all members, compare itimes
        for fid, member in self.get_valid_members():

            # Set our first itime
            if fid == 0:
                itime = member.get_itime()
            else:
                if member.get_itime() != itime:
                    err_msg = 'FID #%d has mismatching integration time!' % fid
                    self.logger.error(err_msg)
                    raise ValueError(err_msg)

        # Finally return the itime
        return itime

    def get_member(self, visibs_ip):

        # Find the member with the right visibs_ip
        for fid, member in self.get_valid_members():

            # Return if we found it
            if member.get_visibs_ip() == visibs_ip:
                return fid, member

    def members_do(self, func):

        # Create empty list for return values
        return_values = []

        # Run func on each valid member
        for fid, member in self.get_valid_members():
            return_values.append(func(fid, member))

        # Return the results, if present
        if any(rv is not None for rv in return_values):
            return return_values

    def set_noise(self, correlation=1.0):

        # Create a common seed
        correlated_bits = int(32 * clip(correlation, 0.0, 1.0))
        uncorrelated_bits = 32 - correlated_bits
        common_seed = randint(0, 2**correlated_bits - 1)

        # Set noise on all inputs
        for fid, member in self.get_valid_members():

            # Make sure all non-correlated nibbles are different
            uncommon_seed_0 = int(('%x' % (2*fid+0)) * 8, 16)
            uncommon_seed_1 = int(('%x' % (2*fid+1)) * 8, 16)

            # Mix our common seed with a random one
            uncommon_mask = ((2**uncorrelated_bits - 1) << correlated_bits)
            seed_0 = (uncommon_seed_0 & uncommon_mask) + common_seed
            seed_1 = (uncommon_seed_1 & uncommon_mask) + common_seed

            # Finall set seeds for this member
            member.set_noise(seed_0, seed_1)

        # Reset the digital noise
        for fid, member in self.get_valid_members():
            member.reset_digital_noise()

    def setup(self, raise_qdr_err=True, threaded=False):

        # Setup each member
        if not threaded:
            for fid, member in self.get_valid_members():
                member.setup(self.qid, fid, self.fids_expected, 0.0, raise_qdr_err=raise_qdr_err)

        else: # if requested, do threaded setup

            # Create our setup threads
            exceptions_queue = Queue()
            setup_threads = OrderedDict()
            for fid, member in self.get_valid_members():
                thread = ExceptingThread(
                    exceptions_queue,
                    target=member.setup,
                    logger=member.logger,
                    kwargs={'raise_qdr_err': raise_qdr_err},
                    args=(self.qid, fid, self.fids_expected, 0.0),
                    )
                setup_threads[thread] = member

            # Now start them all
            for thread in setup_threads.iterkeys():
                thread.start()

            # ...and immediately join them
            for thread in setup_threads.iterkeys():
                thread.join()

            # If there were exceptions log them
            exceptions = 0
            while not exceptions_queue.empty():
                exceptions += 1
                thread, exc = exceptions_queue.get()
                fmt_exc = format_exception(*exc)
                tb_str = ''.join(fmt_exc[0:-1])
                exc_str = ''.join(fmt_exc[-1])
                for line in tb_str.splitlines():
                    setup_threads[thread].logger.debug('<{0}> {1}'.format(exceptions, line))
                for line in exc_str.splitlines():
                    setup_threads[thread].logger.error('<{0}> {1}'.format(exceptions, line))

            # If any exception occurred raise error
            if exceptions > 0:
                raise RuntimeError('{0} member(s) had an error during SwarmQuadrant setup!'.format(exceptions))


class Swarm:

    def __init__(self, map_filenames=[], parent_logger=module_logger, mappings_dict={}):

        # Set initial member variables
        self.logger = parent_logger.getChild(
            '{name}'.format(name=self.__class__.__name__)
            )

        # If map_filenames is empty, look up the current list of active swarm quadrants.
        if not map_filenames:
            mappings_dict = mappings_dict if mappings_dict else self.get_current_swarm_mappings()
            self.quads = list(SwarmQuadrant(k, v, parent_logger=self.logger) for k, v in mappings_dict.items())
        else:
            # Legacy behavior: For every mapping file, instantiate one SwarmQuadrant.
            # Index problems will occur if the map file paths aren't in sequential order.
            self.quads = list(SwarmQuadrant(q, m, parent_logger=self.logger) for q, m in enumerate(map_filenames))

    def __len__(self):
        return len(self.quads)

    def __getitem__(self, qid):
        return self.quads[qid]

    def __repr__(self):
        return '{name}(quads=[{quads}])'.format(name=self.__class__.__name__, quads=self.quads)

    def __str__(self):
        return os.linesep.join(str(quad) for quad in self.quads)

    def get_current_swarm_mappings(self):
        """
        Utility function to read the SWARMQuadrantsInArray file, and based on the contents return
        the appropriate SWARM mapping paths.
        :return: Dictionary of active quadrants where the keys are quadrant numbers and value is mapfile path.
        """
        active_quad_mappings = {}
        try:
            with open(ACTIVE_QUADRANTS_FILE_PATH) as quadrants_file:
                line = quadrants_file.readline().strip()
            if line:
                active_quads = [int(x) for x in line.split(" ")]
                for num in range(1, SWARM_MAX_NUM_QUADRANTS + 1):
                    if num in active_quads:
                        active_quad_mappings[num] = (SWARM_MAPPINGS[num - 1])

        except IOError as e:
            self.logger.error(e)

        finally:
            # If there is a problem reading this file, default to all quadrants active.
            if not active_quad_mappings:
                self.logger.info("Unable to read SWARMQuadrantsInArray file, defaulting to all SWARM quadrants active.")
                active_quad_mappings = SWARM_MAPPINGS
            return active_quad_mappings

    def get_valid_members(self):

        # Return a list of valid fids and members
        for quad in self.quads:
            for tup in quad.get_valid_members():
                yield tup

    def quadrants_do(self, func):

        # Create empty list for return values
        return_values = []

        # Run func on every quadrant
        for qid, quad in enumerate(self.quads):
            return_values.append(func(qid, quad))

        # Return the results, if present
        if any(rv is not None for rv in return_values):
            return return_values

    def __dir__(self):
        return self.__dict__.keys() + dir(Swarm) + dir(SwarmQuadrant) + dir(SwarmMember) + ['roach2', ]

    def __getattr__(self, attr):

        # See if the non-Swarm attribute is a SwarmQuadrant method
        if callable(getattr(self.quads[0], attr, None)):

            # If so, return a callable that passes the users args and kwargs
            # to the appropriate method on all members
            self.logger.debug("{0} is not a Swarm method but it is a SwarmQuadrant method; "
                             "calling on all quadrants!".format(attr))
            return lambda *args, **kwargs: self.quadrants_do(lambda qid, quad: getattr(quad, attr)(*args, **kwargs))

        # See if user is trying to access the SwarmMember.roach2 attribute
        if attr == 'roach2':

            # Define an anonymous class we will instantiate and return
            class AnonClass(object):
                def __dir__(self):
                    return dir(FpgaClient)
                def __getattr__(self_, attr):
                    if callable(getattr(FpgaClient, attr, None)):
                        return lambda *args, **kwargs: self.members_do(lambda fid, member: getattr(member.roach2, attr)(*args, **kwargs))
                    else:
                        return self.members_do(lambda fid, member: getattr(member.roach2, attr))

            return AnonClass()

        else: # Remember to raise an error if nothing is found
            raise AttributeError("{0} not an attribute of Swarm or SwarmQuadrant".format(attr))

    def setup(self, itime, interfaces, delay_test=False, raise_qdr_err=True, threaded=False):

        # Copy interfaces over, and make sure it's a list
        interfaces = list(interfaces[:])

        # Setup each quadrant
        if not threaded:
            for quad in self.quads:
                quad.setup(raise_qdr_err=raise_qdr_err, threaded=False)

        else: # if requested, do threaded setup

            # Create our setup threads
            exceptions_queue = Queue()
            setup_threads = OrderedDict()
            for quad in self.quads:
                thread = ExceptingThread(
                    exceptions_queue,
                    target=quad.setup,
                    kwargs={'raise_qdr_err': raise_qdr_err, 'threaded': True},
                    )
                setup_threads[thread] = quad

            # Now start them all
            for thread in setup_threads.iterkeys():
                thread.start()

            # ...and immediately join them
            for thread in setup_threads.iterkeys():
                thread.join()

            # If there were exceptions log them
            exceptions = 0
            while not exceptions_queue.empty():
                exceptions += 1
                thread, exc = exceptions_queue.get()
                fmt_exc = format_exception(*exc)
                tb_str = ''.join(fmt_exc[0:-1])
                exc_str = ''.join(fmt_exc[-1])
                for line in tb_str.splitlines():
                    setup_threads[thread].logger.debug('<{0}> {1}'.format(exceptions, line))
                for line in exc_str.splitlines():
                    setup_threads[thread].logger.error('<{0}> {1}'.format(exceptions, line))

            # If any exception occurred raise error
            if exceptions > 0:
                raise RuntimeError('{0} quadrant(s) had an error during Swarm setup!'.format(exceptions))

        # Sync the SWARM
        self._sync()

        # Do the post-sync setup
        for fid, member in self.get_valid_members():
            member.set_source(3, 3)
            member.enable_network()

        # Wait for initial accumulations to finish
        self.logger.info('Waiting for initial accumulations to finish...')
        while any(m.roach2.read_uint('xeng_xn_num') for f, m in self.get_valid_members()):
            sleep(0.1)

        # Set the itime and wait for it to register
        self.logger.info('Setting integration time and resetting x-engines...')
        for fid, member in self.get_valid_members():
            member.set_itime(itime)

        # Reset the xengines until window counters to by in sync
        win_period = SWARM_ELEVENTHS * (SWARM_EXT_HB_PER_WCYCLE/SWARM_WALSH_SKIP)
        win_sync = False
        while not win_sync:
            self.reset_xengines()
            sleep(0.5)
            win_count = array([m.roach2.read_uint('xeng_status') for f, m in self.get_valid_members()])
            win_sync = len(set(c/win_period for c in win_count)) == 1
            self.logger.info('Window sync: {0}'.format(win_sync))

        # Reset digital noise
        self.reset_digital_noise()

        # Setup the visibility outputs per quad
        listener = data.SwarmListener('lo') # default to loopback
        for qid, quad in enumerate(self.quads):

            # Pop an interface (should be at least one)
            try:
                listener = data.SwarmListener(interfaces.pop(0))
            except IndexError:
                self.logger.debug('Reached end of interface list; using last given')

            # We've got a listener, setup this quadrant
            for fid, member in quad.get_valid_members():
                member.setup_visibs(qid, listener, delay_test=delay_test)

    def sync(self):

        # Check to make sure corner-turn is in nominal state
        for fid, member in self.get_valid_members():
            if member.roach2.read_uint(SWARM_NETWORK_CTRL) != 0x00000030:
                err_msg = "Corner-turn is in unknown state! Aborting sync"
                member.logger.error(err_msg)
                raise ValueError(err_msg)

        # Disable corner-turn Rx
        self.logger.info('Disabling the corner-turn Rx')
        for fid, member in self.get_valid_members():
            member.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x00000020))
        sleep(5)

        # Disable corner-turn Tx
        self.logger.info('Disabling the corner-turn Tx')
        for fid, member in self.get_valid_members():
            member.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x00000000))
        sleep(5)

        # Do the usual sync operation
        self._sync()

        # Wait for a long while here
        self.logger.info('Waiting for 1 minute; please be patient')
        sleep(60)

        # Reset the corner-turn
        self.logger.info('Resetting the corner-turn Rx')
        for fid, member in self.get_valid_members():
            member.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x40000000))
        sleep(5)

        # Enable the corner-turn Tx
        self.logger.info('Enabling the corner-turn Tx')
        for fid, member in self.get_valid_members():
            member.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x00000020))
        sleep(5)

        # Enable the corner-turn Rx
        self.logger.info('Enabling the corner-turn Rx')
        for fid, member in self.get_valid_members():
            member.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x00000030))
        sleep(5)

        # Finally reset the DDR3 just in case
        self.logger.info('And finally resetting the DDR3')
        for fid, member in self.get_valid_members():
            member.reset_ddr3()

    def _sync(self):

        # Do a threaded sync of SOWF
        sowf_threads = list(Thread(target=m.sync_sowf) for f, m in self.get_valid_members())
        for thread in sowf_threads:
            thread.start()
        self.logger.info('SOWF sync attempted')
        sleep(1)

        # Do a threaded sync of 1PPS
        pps_threads = list(Thread(target=m.sync_1pps) for f, m in self.get_valid_members())
        for thread in pps_threads:
            thread.start()
        self.logger.info('1PPS sync attempted')
        sleep(1)

        # Do a threaded sync of MCNT
        mcnt_threads = list(Thread(target=m.sync_mcnt) for f, m in self.get_valid_members())
        for thread in mcnt_threads:
            thread.start()
        self.logger.info('MCNT sync attempted')
        sleep(1)

        # Do a threaded sync of BENG
        beng_threads = list(Thread(target=m.sync_beng) for f, m in self.get_valid_members())
        for thread in beng_threads:
            thread.start()
        self.logger.info('BENG sync attempted')
        sleep(1)

        # Finally join all threads
        for thread in sowf_threads + pps_threads + mcnt_threads + beng_threads:
            thread.join()

    def reset_xengines(self):

        # Do a threaded reset_xeng
        rstxeng_threads = list(Thread(target=m.reset_xeng) for f, m in self.get_valid_members())
        for thread in rstxeng_threads:
            thread.start()
        self.logger.info('Resetting the X-engines')
        sleep(1)

        # Finally join all threads
        for thread in rstxeng_threads:
            thread.join()

    def reset_digital_noise(self):

        # Do a threaded reset_digital_noise
        rstnoise_threads = list(Thread(target=m.reset_digital_noise) for f, m in self.get_valid_members())
        for thread in rstnoise_threads:
            thread.start()
        self.logger.info('Resetting the digital noise')

        # Finally join all threads
        for thread in rstnoise_threads:
            thread.join()

    def setup_beamformer(self):

        # Do a threaded sync_beng
        beng_threads = list(Thread(target=m.sync_beng) for f, m in self.get_valid_members())
        for thread in beng_threads:
            thread.start()
        self.logger.info('Synced the B-engines')
        sleep(1)

        # Do a threaded dbe.sync_rtime
        rtime_threads = list(Thread(target=dbe.sync_rtime, args=(m, )) for f, m in self.get_valid_members())
        for thread in rtime_threads:
            thread.start()
        self.logger.info('Synced the rtime generators')
        sleep(1)

        # Join all the threads
        for thread in beng_threads + rtime_threads:
            thread.join()

        # Finally setup the individual quadrant beamformers
        for quad in self.quads:
            quad.setup_beamformer()
        self.logger.info('Configured the beamformers')

    def set_beamformer_inputs(self, inputs=None):

        # Go through all quadrants get inputs
        for quad in self.quads:
            quad.set_beamformer_inputs(inputs[quad.qid] if inputs else ())

    def get_itime(self):

        itime = None

        # Go through all quadrants compare itimes
        for quad in self.quads:

            # Set our first itime
            if quad.qid == 0:
                itime = quad.get_itime()
            else:
                if quad.get_itime() != itime:
                    err_msg = 'QID #%d has mismatching integration time!' % quad.qid
                    self.logger.error(err_msg)
                    raise ValueError(err_msg)

        # Finally return the itime
        return itime

    def get_delay(self, this_input):

        # Make sure we've been given a SwarmInput
        if not isinstance(this_input, SwarmInput):
            raise ValueError("Function requires a SwarmInput object!")

        # Loop through each valid member
        delays_found = []
        members_found = 0
        for fid, member in self.get_valid_members():

            # And loop through each input
            for input_n, input_inst in enumerate(member._inputs):

                # Does this one have it?
                if this_input == input_inst:
                    delays_found.append(member.get_delay(input_n))
                    members_found += 1

        # Return different values depending on how many instances found
        if members_found == 0:
            self.logger.error('{} not in SWARM!'.format(this_input))
            return None
        elif members_found == 1:
            return delays_found[0]
        else:
            return delays_found

    def set_delay(self, this_input, value):

        # Make sure we've been given a SwarmInput
        if not isinstance(this_input, SwarmInput):
            raise ValueError("Function requires a SwarmInput object!")

        # Loop through each valid member
        members_found = 0
        for fid, member in self.get_valid_members():

            # And loop through each input
            for input_n, input_inst in enumerate(member._inputs):

                # Does this one have it?
                if this_input == input_inst:
                    member.set_delay(input_n, value)
                    members_found += 1

        # Return different values depending on how many instances found
        if members_found == 0:
            self.logger.error('{} not in SWARM!'.format(this_input))

    def get_phase(self, this_input):

        # Make sure we've been given a SwarmInput
        if not isinstance(this_input, SwarmInput):
            raise ValueError("Function requires a SwarmInput object!")

        # Loop through each valid member
        phases_found = []
        members_found = 0
        for fid, member in self.get_valid_members():

            # And loop through each input
            for input_n, input_inst in enumerate(member._inputs):

                # Does this one have it?
                if this_input == input_inst:
                    phases_found.append(member.get_phase(input_n))
                    members_found += 1

        # Return different values depending on how many instances found
        if members_found == 0:
            self.logger.error('{} not in SWARM!'.format(this_input))
            return None
        elif members_found == 1:
            return phases_found[0]
        else:
            return phases_found

    def set_phase(self, this_input, value):

        # Make sure we've been given a SwarmInput
        if not isinstance(this_input, SwarmInput):
            raise ValueError("Function requires a SwarmInput object!")

        # Loop through each valid member
        members_found = 0
        for fid, member in self.get_valid_members():

            # And loop through each input
            for input_n, input_inst in enumerate(member._inputs):

                # Does this one have it?
                if this_input == input_inst:
                    member.set_phase(input_n, value)
                    members_found += 1

        # Return different values depending on how many instances found
        if members_found == 0:
            self.logger.error('{} not in SWARM!'.format(this_input))

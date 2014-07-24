import logging
from time import sleep
from struct import pack, unpack
from random import randint
from socket import inet_ntoa
from threading import Thread, Event
from collections import OrderedDict

from numpy import clip, roll

from corr.katcp_wrapper import FpgaClient
from katcp import Message

from adc5g import (
    calibrate_mmcm_phase,
    unset_test_mode,
    set_test_mode,
    sync_adc,
    )

from defines import *
from xeng import SwarmXengine


class SwarmInput:

    def __init__(self, antenna=None, chunk=None, polarization=None):

        # Set all initial members
        self.logger = logging.getLogger('SwarmMember')
        self._ant = antenna
        self._chk = chunk
        self._pol = polarization

    def __repr__(self):
        repr_str = 'SwarmInput(antenna={ant}, chunk={chk}, polarization={pol})' 
        return repr_str.format(ant=self._ant, chk=self._chk, pol=self._pol)

    def __str__(self):
        repr_str = 'ant{ant}:chk{chk}:pol{pol}'
        return repr_str.format(ant=self._ant, chk=self._chk, pol=self._pol)

    def __hash__(self):
        return hash((self._ant, self._chk, self._pol))

    def __eq__(self, other):
        if other is not None:
            return (self._ant, self._chk, self._pol) == (other._ant, other._chk, other._pol)
        else:
            return not self.is_valid()

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_valid(self):
        return (self._ant != None) and (self._chk != None) and (self._pol != None)


class SwarmMember:

    def __init__(self, roach2_host):

        # Set all initial members
        self.logger = logging.getLogger('SwarmMember')
        self._inputs = [SwarmInput(),] * len(SWARM_MAPPING_INPUTS)
        self.roach2_host = roach2_host

        # Connect to our ROACH2
        if self.roach2_host:
            self._connect(roach2_host)

    def __eq__(self, other):
        if other is not None:
            return self.roach2_host == other.roach2_host
        else:
            return not self.is_valid()

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_valid(self):
        return self.roach2_host is not None

    def __repr__(self):
        repr_str = 'SwarmMember(roach2_host={host})[{inputs[0]!r}][{inputs[1]!r}]' 
        return repr_str.format(host=self.roach2_host, inputs=self._inputs)

    def __str__(self):
        repr_str = '{host} [{inputs[0]!s}] [{inputs[1]!s}]' 
        return repr_str.format(host=self.roach2_host, inputs=self._inputs)

    def __getitem__(self, input_n):
        return self._inputs[input_n]

    def get_input(self, input_n):
        return self._inputs[input_n]

    def set_input(self, input_n, input_inst):
        self._inputs[input_n] = input_inst

    def setup(self, fid, fids_expected, bitcode, itime_sec, listener, noise=randint(0, 15)):

        # Reset logger for current setup
        self.logger = logging.getLogger('SwarmMember[%d]' % fid)

        # Program the board
        self._program(bitcode)

        # Set noise to perfect correlation
        self.set_noise(0xffffffff, 0xffffffff)
        self.reset_digital_noise()

        # ...but actually use the ADCs
        self.set_source(2, 2)

        # Setup our scopes to capture raw data
        self.set_scope(3, 0, 6)

        # Calibrate the ADC MMCM phases
        self.calibrate_adc()

        # Setup the F-engine
        self._setup_fengine()

        # Setup flat complex gains
        self.set_flat_cgains(0, 2**12)
        self.set_flat_cgains(1, 2**12)

        # Setup the X-engine
        self._setup_xeng_tvg()
        self.set_itime(itime_sec)
        self.reset_xeng()

        # Initial setup of the switched corner-turn
        self._setup_corner_turn(fid, fids_expected)

        # Setup the 10 GbE visibility
        self._setup_visibs(listener)

        # Verify QDRs
        self.verify_qdr()

    def _connect(self, roach2_host):

        # Connect and wait until ready
        self.roach2 = FpgaClient(roach2_host)
        if roach2_host:
            self.roach2.wait_connected()

    def _program(self, bitcode):

        # Program with the bitcode
        self._bitcode = bitcode
        self.roach2.progdev(self._bitcode)

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
            if opt:
                self.logger.info('ADC%d calibration found optimal phase: %d' % (inp, opt))
            else:
                self.logger.error('ADC%d calibration failed!' % inp)

        # Unset test modes
        for inp in SWARM_MAPPING_INPUTS:
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

    def reset_xeng(self):

        # Twiddle bit 29
        mask = 1 << 29 # reset bit location
        val = self.roach2.read_uint(SWARM_XENG_CTRL)
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def get_itime(self):
        
        # Get the integration time in spectra
        xeng_time = self.roach2.read_uint(SWARM_XENG_CTRL) & 0x1ffff
        cycles = xeng_time / (11 * (SWARM_EXT_HB_PER_WCYCLE/SWARM_WALSH_SKIP))
        return cycles * SWARM_WALSH_PERIOD

    def set_itime(self, itime_sec):

        # Set the integration (11 spectra per step * steps per cycle)
        self._xeng_itime = 11 * (SWARM_EXT_HB_PER_WCYCLE/SWARM_WALSH_SKIP) * int(itime_sec/SWARM_WALSH_PERIOD)
        self.roach2.write(SWARM_XENG_CTRL, pack(SWARM_REG_FMT, self._xeng_itime))

    def _reset_corner_turn(self):

        # Twiddle bits 31 and 30
        mask = (1 << 31) + (1 << 30)
        val = self.roach2.read_uint(SWARM_NETWORK_CTRL)
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, val & ~mask))
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, val |  mask))
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, val & ~mask))

    def _setup_corner_turn(self, this_fid, fids_expected, ipbase=0xc0a88000, macbase=0x000f530cd500, bh_mac=0x000f530cd899):

        # Reset the cores
        self._reset_corner_turn()

        # Store our FID 
        self.fid = this_fid
        self.fids_expected = fids_expected

        # Set static parameters
        self.roach2.write_int(SWARM_NETWORK_FIDS_EXPECTED, self.fids_expected)
        self.roach2.write_int(SWARM_NETWORK_IPBASE, ipbase)
        self.roach2.write_int(SWARM_NETWORK_FID, self.fid)

        # Initialize the ARP table 
        arp = [bh_mac] * 256

        # Fill the ARP table
        for fid in SWARM_ALL_FID:
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

    def visibs_delay(self, enable=True, delay_test=False, chunk_delay=2**23):

        # Disable/enable Laura's DDR3 delay and test
        self.roach2.write_int(SWARM_VISIBS_DELAY_CTRL, (enable<<31) + (delay_test<<29) + chunk_delay)

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

    def verify_qdr(self):
  
        # check qdr ready, reset if not ready 
        for qnum in SWARM_ALL_QDR:
            self.logger.debug('checking QDR%d' % qnum)
            rdy = self.qdr_ready(qnum)
            if not rdy:
                self.logger.warning('QDR%d not ready, resetting' % qnum)
                self.reset_qdr(qnum) 
    
    def _setup_visibs(self, listener, delay_test=False):

        # Store (or override) our listener
        self._listener = listener

        # Reset the DDR3
        self.reset_ddr3()

        # Enable DDR3 interleaver
        self.visibs_delay(enable=True)

        # Fill the visibs ARP table
        arp = [0xffffffffffff] * 256
        arp[self._listener.ip & 0xff] = self._listener.mac

        # Configure the transmit interface
        final_hex = (self.fid + 4) * 2
        src_ip = (192<<24) + (168<<16) + (10<<8) + final_hex + 50 
        src_mac = (2<<40) + (2<<32) + final_hex + src_ip
        self.roach2.config_10gbe_core(SWARM_VISIBS_CORE, src_mac, src_ip, 4000, arp)

        # Configure the visibility packet buffer
        self.roach2.write(SWARM_VISIBS_SENDTO_IP, pack(SWARM_REG_FMT, self._listener.ip))
        self.roach2.write(SWARM_VISIBS_SENDTO_PORT, pack(SWARM_REG_FMT, self._listener.port))

        # Reset (and disable) visibility transmission
        self.roach2.write(SWARM_VISIBS_TENGBE_CTRL, pack(SWARM_REG_FMT, 1<<30))
        self.roach2.write(SWARM_VISIBS_TENGBE_CTRL, pack(SWARM_REG_FMT, 0))

        # Finally enable transmission
        self.roach2.write(SWARM_VISIBS_TENGBE_CTRL, pack(SWARM_REG_FMT, 1<<31))

    def get_visibs_ip(self):

        # Update/store the visibs core net info
        self.visibs_netinfo = self.roach2.get_10gbe_core_details(SWARM_VISIBS_CORE)

        # Return the visibs core IP 
        return inet_ntoa(pack(SWARM_REG_FMT, self.visibs_netinfo['my_ip']))

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

    def enable_network(self):

        # Enable the RX and TX
        self.roach2.write(SWARM_NETWORK_CTRL, pack(SWARM_REG_FMT, 0x30))

    def fringe_stop(self, enable):

        # Stop fringe stopping
        message = Message.request(SWARM_FSTOP_STOP_CMD)
        reply, informs = self.roach2.blocking_request(message, timeout=60)
        if not reply.reply_ok():
            self.logger.error("Stopping fringe stopping failed!")

        # Start it again (if requested)
        if enable:
            message = Message.request(SWARM_FSTOP_START_CMD)
            reply, informs = self.roach2.blocking_request(message, timeout=60)
            if not reply.reply_ok():
                self.logger.error("Starting fringe stopping failed!")

    def dewalsh(self, enable_0, enable_1):

        # Set the Walsh control register
        self.roach2.write(SWARM_WALSH_CTRL, pack(SWARM_REG_FMT, (enable_1<<30) + (enable_0<<28) + 0xfffff))

    def set_walsh_pattern(self, input_n, pattern, offset=0, swap90=True):

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
                if swap90:
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

        # Get the delay value in ns
        message = Message.request(SWARM_DELAY_GET_CMD, str(input_n))
        reply, informs = self.roach2.blocking_request(message, timeout=60)
        if not reply.reply_ok():
            self.logger.error("Getting the delay failed!")
        else:
            return float(reply.arguments[1])

    def set_delay(self, input_n, value):

        # Set the delay value in ns
        message = Message.request(SWARM_DELAY_SET_CMD, str(input_n), str(value))
        reply, informs = self.roach2.blocking_request(message, timeout=60)
        if not reply.reply_ok():
            self.logger.error("Setting the delay failed!")


EMPTY_MEMBER = SwarmMember(None)

class Swarm:

    def __init__(self, map_filename=SWARM_MAPPING, walsh_filename=SWARM_WALSH_PATTERNS):

        # Set initial member variables
        self.logger = logging.getLogger('Swarm')

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

    def load_mapping(self, map_filename):

        # Clear the members instance
        self.members = OrderedDict()

        # Store (or restore) current mapping file
        self.map_filename = map_filename

        # Open the mapping file
        with open(self.map_filename, 'r') as map_file:

            # Parse line by line
            for map_line in map_file:

                # Splits by (and removes) whitespace
                entry = map_line.split()

                # Checks if this line is a comment
                is_comment = entry[0].startswith(SWARM_MAPPING_COMMENT)

                if is_comment:

                    # Display map comment
                    self.logger.debug('Mapping comment found: %s' % map_line.rstrip())

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
                    roach2_num = int(entry[0])
                    roach2_host = SWARM_ROACH2_IP % roach2_num

                    # Create and attach our member instance
                    member_inst = SwarmMember(roach2_host)
                    if not self.members.has_key(roach2_host):
                        self.members[roach2_host] = member_inst

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
                    input_inst = SwarmInput(antenna=antenna, chunk=chunk, polarization=pol)
                    self.members[roach2_host].set_input(roach2_inp, input_inst)

                    # We're done, spit some info out
                    self.logger.debug('Mapping antenna=%r to %s:input=%d', 
                                      input_inst, roach2_host, roach2_inp)

        # Set number FIDs expected
        self.fids_expected = len(self.members)

        # Fill missing FIDs with empty members
        missing_fids = set(SWARM_ALL_FID) - set(range(self.fids_expected))
        for fid in missing_fids:
            self.members[fid] = SwarmMember(None)

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

                # Checks if this line is a comment
                is_comment = entry[0].startswith(SWARM_MAPPING_COMMENT)

                if is_comment:

                    # Display map comment
                    self.logger.debug('Mapping comment found: %s' % map_line.rstrip())

                else:

                    # First column is antenna number
                    ant = int(entry[0])

                    # Second column is the pattern
                    pattern = entry[1]

                    # Display Walsh pattern we found
                    self.logger.debug('Length %d Walsh pattern found for antenna #%d' % (len(pattern), ant))

                    # Add it to your patterns
                    self.walsh_patterns[ant] = pattern

    def set_walsh_patterns(self, offset=0, swap90=[True, False]):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Go through every member
        for fid, member in enumerate(valid_members):

            # And every input
            for inp in SWARM_MAPPING_INPUTS:

                # Get the antenna for this input
                ant = member[inp]._ant

                # Get the pattern for the antenna
                pattern = self.walsh_patterns[ant]

                # Then set it using the member function
                member.set_walsh_pattern(inp, pattern, offset=offset, swap90=swap90[inp])

            # Enable de-Walshing
            member.dewalsh(enable_0=3, enable_1=3)

    def load_sideband_states(self):

        # Initialize sideband states
        self.sideband_states = list()

        # Get the Xengine output order
        order = list(SwarmXengine(self).xengine_order())

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
                    self.sideband_states.append(0)
                else:

                    # Get 90/270 Walsh state for both antennas
                    state_left = int(self.walsh_patterns[word.left._ant][step]) & 1
                    state_right = int(self.walsh_patterns[word.right._ant][step]) & 1

                    # Append the combined state
                    self.sideband_states.append(int(state_left == state_right))

    def set_sideband_states(self):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Go through every member
        for fid, member in enumerate(valid_members):

            # Write the states to the FPGA
            member.set_sideband_states(self.sideband_states)

    def fringe_stopping(self, enable):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Go through every member
        for fid, member in enumerate(valid_members):

            # En(dis)able fringe stoppiner per member
            member.fringe_stop(enable)

    def get_itime(self):

        itime = None

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Find the member with the right visibs_ip
        for fid, member in enumerate(valid_members):

            # Set our first itime
            if fid == 0:
                itime = member.get_itime()
            else:
                if member.get_itime() != itime:
                    self.logger.error('FID #%d has mismatching integration time!' % fid)

        # Finally return the itime
        return itime

    def get_member(self, visibs_ip):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Find the member with the right visibs_ip
        for fid, member in enumerate(valid_members):

            # Return if we found it
            if member.get_visibs_ip() == visibs_ip:
                return fid, member

    def members_do(self, func):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Run func on each valid member
        for fid, member in enumerate(valid_members):
            func(fid, member)

    def set_noise(self, correlation=1.0):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Create a common seed
        correlated_bits = int(32 * clip(correlation, 0.0, 1.0))
        uncorrelated_bits = 32 - correlated_bits
        common_seed = randint(0, 2**correlated_bits - 1)

        # Set noise on all inputs
        for fid, member in enumerate(valid_members):

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
        for fid, member in enumerate(valid_members):
            member.reset_digital_noise()

    def get_delay(self, antenna, chunk=0, polarization=0):

        # Create an input instance
        this_input = SwarmInput(antenna=antenna, chunk=chunk, polarization=polarization)

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Loop through each valid member
        delays_found = []
        members_found = 0
        for fid, member in enumerate(valid_members):

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

    def set_delay(self, antenna, chunk=0, polarization=0, value=0.0):

        # Create an input instance
        this_input = SwarmInput(antenna=antenna, chunk=chunk, polarization=polarization)

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Loop through each valid member
        members_found = 0
        for fid, member in enumerate(valid_members):

            # And loop through each input
            for input_n, input_inst in enumerate(member._inputs):

                # Does this one have it?
                if this_input == input_inst:
                    member.set_delay(input_n, value)
                    members_found += 1

        # Return different values depending on how many instances found
        if members_found == 0:
            self.logger.error('{} not in SWARM!'.format(this_input))

    def sync(self):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Sync SOWF and 1PPS
        for fid, member in enumerate(valid_members):
            member.sync_sowf()
            member.sync_1pps()

        # Wait for the sync
        self.logger.info('SOWF and 1PPS sync attempted')
        sleep(10)

        # Sync MCNT 
        for fid, member in enumerate(valid_members):
            member.sync_mcnt()

        # Wait for the MCNT sync
        self.logger.info('MCNT sync attempted')
        sleep(10)

    def setup(self, bitcode, itime, listener):

        # Create list of valid members
        valid_members = list(self[fid] for fid in range(self.fids_expected))

        # Go through hosts in our mapping
        for fid, member in enumerate(valid_members):

            # Log some transmit side (i.e. ROACH2) network information
            self.logger.info('Configuring ROACH2=%s for transmission as FID #%d' %(member.roach2_host, fid))

            # Setup (i.e. program and configure) the ROACH2
            member.setup(fid, self.fids_expected, bitcode, itime, listener)

        # Sync the SWARM
        self.sync()

        # Do the post-sync setup
        for fid, member in enumerate(valid_members):
            member.enable_network()
            member.reset_xeng()
            member.reset_digital_noise()
            member.set_source(3, 3)

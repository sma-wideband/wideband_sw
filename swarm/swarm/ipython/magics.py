import logging
from IPython.core.magic import (
    Magics, 
    magics_class, 
    line_magic,
    cell_magic, 
    line_cell_magic
    )

from swarm.core import SwarmInput
from swarm.defines import *

@magics_class
class SwarmShellMagics(Magics):

    def __init__(self, shell, swarm):
        self.logger = logging.getLogger(self.__class__.__name__)
        super(SwarmShellMagics, self).__init__(shell)
        self.swarm = swarm

    @line_magic
    def config(self, line):

        if line.lower() == 'dual-rx':

            # Set quadrant one to be dual-Rx, 8-10 GHz (no flip)
            self.swarm[0].members_do(lambda fid, mem: setattr(mem, 'dc_if_freqs', [7.85, 7.85]))

            # Enable fringe stopping first
            self.swarm[0].fringe_stopping(True)

            # Reset all Walshing and sideband separation tables
            self.swarm[0].set_walsh_patterns()
            self.swarm[0].set_sideband_states()

            # Alert user of any other tasks
            self.logger.info("Done. If you have not done so already (after a setIFLevels), "
                             "please remember to run *all* BDC scripts with the 'b' option.")

        elif line.lower() == 'single-rx':

            # Set quadrant one to be single-Rx, 8-12 GHz (no flip in chunk 0, flip in chunk 1)
            self.swarm[0].members_do(lambda fid, mem: setattr(mem, 'dc_if_freqs', [7.85, -12.15]))

            # Enable fringe stopping first
            self.swarm[0].fringe_stopping(True)

            # Reset all Walshing and sideband separation tables
            self.swarm[0].set_walsh_patterns()
            self.swarm[0].set_sideband_states()

            # Alert user of any other tasks
            self.logger.info("Done. If you have not done so already (after a setIFLevels), "
                             "please remember to run *both* BDC scripts with the 'l' or 'h' option.")

        else:
            self.logger.error("Config \"{0}\" not supported!".format(line))

    @line_magic
    def calibrate(self, line):

        if line.lower() == 'adc-warm':

            # Disable the ADC monitor
            self.swarm.send_katcp_cmd('stop-adc-monitor')

            # Do the warm ADC cal
            self.swarm.warm_calibrate_adc()

            # Enable the ADC monitor
            self.swarm.send_katcp_cmd('start-adc-monitor')

        else:
            self.logger.error("Calibration \"{0}\" not supported!".format(line))

    @line_magic
    def showbeam(self, line):

        # Read SWARM beamformer configuration
        inputs = self.swarm.get_beamformer_inputs()
        gains = self.swarm.get_bengine_gains()
        for ii,q in enumerate(inputs):
            m = gains[ii][0]
            print "qid{0}: gains = (RxA-USB = {1}, RxB-USB = {2}, RxA-LSB = {3}, RxB-LSB = {4})".format(ii,m[0],m[1],m[2],m[3])
            for ii,sb in enumerate(q.keys()):
                print "  |"
                print "  +--{0}".format(sb)
                for inp in q[sb]:
                    if ii+1 < len(q.keys()):
                        print "  |  +--{0}".format(inp)
                    else:
                        print "     +--{0}".format(inp)

    @line_magic
    def setbeam(self, line):

        # Parse arguments
        gain = None
        idx_gain = line.find('gain=')
        if idx_gain == -1:
            ant_list = eval(line)
        else:
            ant_list = eval(line[:idx_gain])
            gain_val = line[idx_gain:].split('=')[1]
            if len(gain_val) > 0:
                gain = eval(gain_val)

        # Build beams structure
        beams = [dict(zip(SWARM_BENGINE_SIDEBANDS,[[]]*len(SWARM_BENGINE_SIDEBANDS))),] * len(SWARM_MAPPING_CHUNKS)
        for quad in self.swarm.quads:
            qid = quad.qid
            if hasattr(quad,'sdbe'):
                sdbe = getattr(quad,'sdbe')
                for sb in SWARM_BENGINE_SIDEBANDS:
                    if sdbe.find(sb) != -1:
                        beams[qid][sb] = [SwarmInput(ant, qid, pol) for pol in SWARM_MAPPING_POLS for ant in ant_list]

        # Apply beams structure to SWARM
        self.swarm.set_beamformer_inputs(beams)

        # Maximum allowed gain value
        MAX_GAIN = 15.9375
        MIN_GAIN = 0.0

        # If manual gain value not specified, set gains inversely proportional to the number of antennas in beam
        if gain is None:
            gain = MAX_GAIN / (len(ant_list))
        elif gain > MAX_GAIN:
            gain = MAX_GAIN
        elif gain < MIN_GAIN:
            gain = MIN_GAIN

        # Apply gain
        self.swarm.set_bengine_gains(gain, gain, gain, gain)

    @line_magic
    def setprompt(self, line):
        self.shell.in_template = line

    #@cell_magic
    def cmagic(self, line, cell):
        "my cell magic"
        return line, cell

    #@line_cell_magic
    def lcmagic(self, line, cell=None):
        "Magic that works both as %lcmagic and as %%lcmagic"
        if cell is None:
            print "Called as line magic"
            return line
        else:
            print "Called as cell magic"
            return line, cell

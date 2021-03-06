from redis import StrictRedis
from numpy import array, conjugate, exp, pi, vstack, zeros
from swarm.pysendint import send_integration
from swarm import SwarmDataCallback
from swarm.data import SwarmDataPackage

REDIS_UNIX_SOCKET = '/tmp/redis.sock'


class SMAData(SwarmDataCallback):

    def __init__(self, swarm, redis_host='localhost', redis_port=6379, pub_channel='swarm.data', rephase_2nd_sideband_data=False):
        self.redis = StrictRedis(redis_host, redis_port, unix_socket_path=REDIS_UNIX_SOCKET)
        self.rephase_2nd_sideband_data = rephase_2nd_sideband_data
        super(SMAData, self).__init__(swarm)
        self.pub_channel = pub_channel

    def __call__(self, data):
        """ Callback for sending data to SMA's dataCatcher/corrSaver """

        # Apply beamformer second sideband phases if needed
        if self.rephase_2nd_sideband_data:
            data = self.apply_beamformer_second_sideband_phase(data)

            # Debug log that second sideband phases applied
            self.logger.debug("Applied 2nd sideband beamformer phases to correlator data")

        # Publish the raw data to redis
        subs = self.redis.publish(self.pub_channel, data)

        # Info log the set
        self.logger.info("Data sent to %d subscribers", subs)

    def apply_beamformer_second_sideband_phase(self, data, sideband="LSB"):

        # Get the phases that need to be applied to each baseline
        left_inputs = [bl.left for bl in data.baselines]
        left_phases = array(self.swarm.get_beamformer_second_sideband_phase(left_inputs))
        right_inputs = [bl.right for bl in data.baselines]
        right_phases = array(self.swarm.get_beamformer_second_sideband_phase(right_inputs))
        bl_phases = exp(-1j * pi/180.0 * (left_phases - right_phases))

        # Apply transformation to each baseline
        new_data = SwarmDataPackage.from_string(str(data))
        for ibl, bl in enumerate(data.baselines):

            # Extract complex correlator data
            baseline_data = new_data[bl, sideband]
            complex_data = baseline_data[0::2] + 1j * baseline_data[1::2]

            # Apply phases
            complex_data = complex_data * bl_phases[ibl]

            # Reformat to original format and store
            baseline_data = vstack((complex_data.real,complex_data.imag)).flatten(order='F')
            new_data[bl, sideband] = baseline_data

        return new_data

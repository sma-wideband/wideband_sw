SWARM_ROACH2_IP = 'roach2-%02x'

SWARM_MAPPING = '/global/configFiles/swarmMapping'
SWARM_MAPPING_COMMENT = '#'
SWARM_MAPPING_COLUMNS = 5
SWARM_MAPPING_INPUTS = (0, 1)
SWARM_MAPPING_CHUNKS = (0, 1)
SWARM_MAPPING_POLS = (0, 1)

SWARM_WALSH_PATTERNS = '/global/configFiles/swarmWalshPatterns'
SWARM_WALSH_TABLE_BRAM = 'walsh_table'
SWARM_WALSH_CTRL = 'walsh_ctrl'
SWARM_WALSH_TABLE_LEN = 1024

SWARM_SB_STATE_BRAM = 'xengine_final_acc_sb_state'

# Clocking parameters
# (changes with external clock)
SWARM_CLOCK_RATE = 156e6

# Overall design parameters
# (should not change unless design changes)
SWARM_CHANNELS = 2**14

# Internal de-Walshing parameters
# (should not change unless design changes)
SWARM_INT_HB_PER_SOWF = 64
SWARM_TRANSPOSE_SIZE = 128
SWARM_CLOCKS_PER_INT_HB = 6 * SWARM_TRANSPOSE_SIZE * 2048
SWARM_INT_HB_PERIOD = SWARM_CLOCKS_PER_INT_HB / SWARM_CLOCK_RATE

# External Walshing parameters
# (changes with SMA Walshing changes)
SWARM_EXT_HB_PER_WCYCLE = 64
SWARM_EXT_HB_PERIOD = (2**19) / (52e6)
SWARM_WALSH_PERIOD = SWARM_EXT_HB_PERIOD * SWARM_EXT_HB_PER_WCYCLE

# Internal/external relationship
# (changes with either; see above)
SWARM_WALSH_SKIP = int(SWARM_INT_HB_PERIOD / SWARM_EXT_HB_PERIOD)

SWARM_ALL_FID = range(8)
SWARM_ALL_CORE = range(4)
SWARM_ALL_QDR = range(4)
SWARM_SHIFT_SCHEDULE = 0x555f555f
SWARM_FENGINE_CTRL = 'fengine_ctrl'
SWARM_CGAIN_GAIN = 'cgain_gain_%d'
SWARM_XENG_CTRL = 'xeng_ctrl'
SWARM_XENG_XN_NUM = 'xeng_xn_num'
SWARM_XENG_TVG = 'xengine_xeng_tvg_data%d'
SWARM_VISIBS_CORE = 'visibs_gbe0'
SWARM_VISIBS_DELAY_CTRL = 'visibs_delay_ctrl'
SWARM_VISIBS_TENGBE_CTRL = 'visibs_tengbe_ctrl'
SWARM_VISIBS_SENDTO_IP = 'visibs_sendto_ip'
SWARM_VISIBS_SENDTO_PORT = 'visibs_sendto_port'
SWARM_SOURCE_SEED = 'source_seed_%d'
SWARM_SOURCE_CTRL = 'source_ctrl'
SWARM_QDR_CTRL = 'qdr%d_ctrl'
SWARM_SYNC_CTRL = 'sync_ctrl'
SWARM_SCOPE_CTRL = 'scope_ctrl'
SWARM_SCOPE_SNAP = 'scope_snap%d'
SWARM_NETWORK_IPBASE = 'network_ipbase'
SWARM_NETWORK_CORE = 'network_eth_%d_core'
SWARM_NETWORK_CTRL = 'network_ctrl'
SWARM_NETWORK_FID = 'network_fid'
SWARM_NETWORK_FIDS_EXPECTED = 'network_fids_expected'
SWARM_BENGINE_CORE = 'beng_eth_core'
SWARM_BENGINE_CTRL = 'beng_eth_ctrl'
SWARM_BENGINE_SENDTO_IP = 'beng_eth_dest_ip'
SWARM_REG_FMT = '>I'

SWARM_SYNC_ARM_CMD = 'sma-walsh-arm'
SWARM_FSTOP_STOP_CMD = 'sma-astro-fstop-stop'
SWARM_FSTOP_START_CMD = 'sma-astro-fstop-start'
SWARM_DELAY_GET_CMD = 'sma-astro-delay-get'
SWARM_DELAY_SET_CMD = 'sma-astro-delay-set'

SWARM_XENG_PARALLEL_CHAN = 8
SWARM_XENG_SIDEBANDS = ('LSB', 'USB')
SWARM_XENG_TOTAL = SWARM_XENG_PARALLEL_CHAN * len(SWARM_ALL_FID)

SWARM_VISIBS_N_PKTS = 512
SWARM_VISIBS_CHANNELS = 2048
SWARM_VISIBS_BASELINES = 128
SWARM_VISIBS_HEADER_SIZE = 8
SWARM_VISIBS_HEADER_FMT = '>II'
SWARM_VISIBS_PKT_SIZE = 1025 * 8
SWARM_VISIBS_SIDEBANDS = len(SWARM_XENG_SIDEBANDS)
SWARM_VISIBS_ACC_SIZE = SWARM_VISIBS_CHANNELS * SWARM_VISIBS_BASELINES * SWARM_VISIBS_SIDEBANDS * 2

SWARM_BENGINE_CORE = 'beng_eth_core'
SWARM_BENGINE_CTRL = 'beng_eth_ctrl'
SWARM_BENGINE_SENDTO_IP = 'beng_eth_dest_ip'
SWARM_BENGINE_PKT_SIZE = 2056
SWARM_BENGINE_HEADER_SIZE = 8
SWARM_BENGINE_HEADER_FMT = '>BIBH'
SWARM_BENGINE_PAYLOAD_SIZE = SWARM_BENGINE_PKT_SIZE - SWARM_BENGINE_HEADER_SIZE

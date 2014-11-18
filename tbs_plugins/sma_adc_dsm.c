#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <sys/stat.h>
#include <katcp.h>
#include <sys/mman.h>
#include <tcpborphserver3.h>
#include <dsm.h>
#include <pthread.h>
#include <plugin.h>
#include <sma_adc.h>

#define TRUE 1
#define FALSE 0
#define FAIL -1

#define SCOPE_0_DATA "scope_snap0_bram"
#define SCOPE_0_CTRL "scope_snap0_ctrl"
#define SCOPE_0_STATUS "scope_snap0_status"
#define SCOPE_1_DATA "scope_snap1_bram"
#define SCOPE_1_CTRL "scope_snap1_ctrl"
#define SCOPE_1_STATUS "scope_snap1_status"

/* pointers for controlling and accessing the snapshot */
static int *ctrl_p[2], *stat_p[2];
static signed char *snap_p[2];
pthread_mutex_t snap_mutex;

/* the following are for programming the adc through the spi interface. */
#define ADC5G_CONTROLLER "adc5g_controller"
#define CONTROL_REG_ADDR 0x01
#define CHANSEL_REG_ADDR 0x0f
#define EXTOFFS_REG_ADDR 0x20
#define EXTGAIN_REG_ADDR 0x22
#define EXTPHAS_REG_ADDR 0x24
#define CALCTRL_REG_ADDR 0x10
#define FIRST_EXTINL_REG_ADDR 0x30
#define SPI_WRITE 0x80;

/* Pointer for accessing the spi interfrace */
uint32_t *spi_p;

/* Default file names */
#define SNAP_NAME "/instance/adcTests/snap"
#define OGP_BASE_NAME "/instance/configFiles/ogp_if"

/* The ogp array contains o, g and p of core A followed by the same for cores
 * B, C and D.
 * In this case, since phase can not be determined, each p will be 0.
 * Overlaod_cnt will contain a count of -128 and 127 codes for each core.
 */
typedef struct {
  float offs[4];
  float gains[4];
  float avz;
  float avamp;
  int overload_cnt[4];
} og_rtn;

/* Things for the adc monitor thread */

int run_adc_monitor = FALSE;
int run_cmd_monitor = FALSE;
pthread_t cmd_monitor_thread;
pthread_t adc_monitor_thread;
dsm_structure dsm_adc_cal;
int adc_cal_valid = 0;
int adc_cmd_rtn[3];
#define CMD_STATUS adc_cmd_rtn[0]

struct tbs_raw *get_mode_pointer(struct katcp_dispatch *d){
  struct tbs_raw *tr;

  /* Grab the mode pointer and check that the FPGA is programmed */
  tr = get_mode_katcp(d, TBS_MODE_RAW);
  if(tr == NULL){
    CMD_STATUS = RAW_MODE;
    return NULL;
  }

  /* Make sure we're programmed */
  if(tr->r_fpga != TBS_FPGA_MAPPED){
    CMD_STATUS = FPGA_PGM;
    return NULL;
  }
  return(tr);
}

int set_spi_pointer( struct tbs_raw *tr, int zdok) {
  struct tbs_entry *spi_reg;

  spi_reg = find_data_avltree(tr->r_registers, ADC5G_CONTROLLER);
  if(spi_reg == NULL) {
    CMD_STATUS = SPI_REG;
    return 0;
  }
  spi_p = ((uint32_t *)(tr->r_map + spi_reg->e_pos_base)) + 1 + zdok;
  return 1;
}

/* the following spi subroutines assume that spi_p is set up.  It also
 * assumes that it is running on a big-endian (network byte order) machine
 * like the ppc on a roach2*/

/* return the 16 bit value which was read or -1 if an error */
int get_spi_register(int reg_addr) {
  unsigned char spi_val[4];

  /* request the register */
  spi_val[0] = 0;
  spi_val[1] = 0;
  spi_val[2] = (unsigned char)reg_addr;
  spi_val[3] = 1;
  *spi_p = *(uint32_t *)spi_val;
  /* delay so that the request can complete */
  usleep(1000);
  /* read the return */
  *(uint32_t *)spi_val = *spi_p;
  if(spi_val[2] == (unsigned char)reg_addr) {
    return *(uint16_t *)spi_val;
  } else {
    return(-1);
  }
}

void set_spi_register(int reg_addr, int reg_val) {
  unsigned char spi_val[4];

  *(uint16_t *)spi_val = (uint16_t)reg_val;
  spi_val[2] = (unsigned char)(reg_addr) | 0x80;
  spi_val[3] = 1;
  *spi_p = *(uint32_t *)spi_val;
  /* delay so that this spi command will finish before another can start */
  usleep(1000);
}

float get_offset_register(int chan) {
  int rtn;

  set_spi_register(CHANSEL_REG_ADDR, chan);
  rtn = get_spi_register(EXTOFFS_REG_ADDR);
  return((rtn - 0x80)*(100./255.));
}

int set_offset_register(int chan, float offset) {
  int reg_val;

  reg_val = (int)(((offset < 0)? -0.5: 0.5) + offset*(255./100.)) + 0x80;
  set_spi_register(CHANSEL_REG_ADDR, chan);
  set_spi_register(EXTOFFS_REG_ADDR, reg_val);
  set_spi_register(CALCTRL_REG_ADDR, 2<<2);
  return reg_val;
}

float get_gain_register(int chan) {
  int rtn;

  set_spi_register(CHANSEL_REG_ADDR, chan);
  rtn = get_spi_register(EXTGAIN_REG_ADDR);
  return((rtn - 0x80)*(36./255.));
}

int set_gain_register(int chan, float gain) {
  int reg_val;

  reg_val = (int)(((gain < 0)? -0.5: 0.5) + gain*(255./36.)) + 0x80;
  set_spi_register(CHANSEL_REG_ADDR, chan);
  set_spi_register(EXTGAIN_REG_ADDR, reg_val);
  set_spi_register(CALCTRL_REG_ADDR, 2<<4);
  return reg_val;
}

float get_phase_register(int chan) {
  int rtn;

  set_spi_register(CHANSEL_REG_ADDR, chan);
  rtn = get_spi_register(EXTPHAS_REG_ADDR);
  return((rtn - 0x80)*(28./255.));
}

int set_phase_register(int chan, float phase) {
  int reg_val;

  reg_val = (int)(((phase < 0)? -0.5: 0.5) + phase*(255./28.)) + 0x80;
  set_spi_register(CHANSEL_REG_ADDR, chan);
  set_spi_register(EXTPHAS_REG_ADDR, reg_val);
  set_spi_register(CALCTRL_REG_ADDR, 2<<6);
  return reg_val;
}

void get_ogp_registers(float *ogp) {
  int chan;

  for(chan = 1; chan < 5; chan++) {
    *ogp++ = get_offset_register(chan);
    *ogp++ = get_gain_register(chan);
    *ogp++ = get_phase_register(chan);
  }
}

void set_ogp_registers(float *ogp) {
  int chan;

  for(chan = 1; chan < 5; chan++) {
    set_offset_register(chan, *ogp++);
    set_gain_register(chan, *ogp++);
    set_phase_register(chan, *ogp++);
  }
}

int read_adc_calibrations(void) {
  if(!adc_cal_valid) {
    if(dsm_read("obscon",DSM_CAL_STRUCT,&dsm_adc_cal, NULL) != DSM_SUCCESS){
      CMD_STATUS = DSM_READ;
      return FAIL;
    }
    adc_cal_valid = TRUE;
  }
  return(OK);
}

int write_adc_calibrations(void) {
  if(!adc_cal_valid) {
    CMD_STATUS = DSM_WRITE;
    adc_cmd_rtn[1] = adc_cal_valid;
    return FAIL;
  }
  if(dsm_write(DSM_MONITOR_CLASS,DSM_CAL_STRUCT,&dsm_adc_cal) != DSM_SUCCESS){
    CMD_STATUS = DSM_WRITE;
    return FAIL;
  }
  return OK;
}

int read_ogp_file(struct katcp_dispatch *d, char *fname, float *ogp) {
  FILE *fp;
  int i;

  fp = fopen(fname, "r");
  if(fp == NULL) {
    CMD_STATUS = FILE_OPEN;
    return 0;
  }
  for(i = 0; i < 12; i++) {
    fscanf(fp, "%f", &ogp[i]);
  }
  fclose(fp);
  return 1;
}

int write_ogp_file(struct katcp_dispatch *d, char *fname, float *ogp) {
  FILE *fp;
  int i;

  umask(0000);
  fp = fopen(fname, "w");
  if(fp == NULL) {
    CMD_STATUS = FILE_OPEN;
    return 0;
  }
  for(i = 0; i < 12; i++) {
    fprintf(fp, "%f\n", ogp[i]);
  }
  fclose(fp);
  return 1;
}

void print_ogp(struct katcp_dispatch *d, float *ogp) {
  int chan;

  for(chan = 0; chan < 4; chan++) {
    log_message_katcp(d, KATCP_LEVEL_INFO, NULL,
      "%.4f %.4f %.4f", ogp[3*chan], ogp[3*chan+1], ogp[3*chan+2]);
  }
}

/* Set up pointers to the registers for taking and accesing a snapshot for
 * both zdoks. */
int set_snapshot_pointers(struct tbs_raw *tr) {
  struct tbs_entry *ctrl_reg, *data_reg, *status_reg;

  /* Get the register pointers */
  ctrl_reg = find_data_avltree(tr->r_registers, SCOPE_0_CTRL);
  status_reg = find_data_avltree(tr->r_registers, SCOPE_0_STATUS);
  data_reg = find_data_avltree(tr->r_registers, SCOPE_0_DATA);
  if(ctrl_reg == NULL || status_reg == NULL || data_reg == NULL
      ){
    CMD_STATUS = SNAP_REG;
    return 0;
  }
  snap_p[0] = ((signed char *)(tr->r_map + data_reg->e_pos_base));
  ctrl_p[0] = ((int *)(tr->r_map + ctrl_reg->e_pos_base));
  stat_p[0] = ((int *)(tr->r_map + status_reg->e_pos_base));
    ctrl_reg = find_data_avltree(tr->r_registers, SCOPE_1_CTRL);
    status_reg = find_data_avltree(tr->r_registers, SCOPE_1_STATUS);
    data_reg = find_data_avltree(tr->r_registers, SCOPE_1_DATA);
  if(ctrl_reg == NULL || status_reg == NULL || data_reg == NULL
      ){
    CMD_STATUS = SNAP_REG;
    return 0;
  }
  snap_p[1] = ((signed char *)(tr->r_map + data_reg->e_pos_base));
  ctrl_p[1] = ((int *)(tr->r_map + ctrl_reg->e_pos_base));
  stat_p[1] = ((int *)(tr->r_map + status_reg->e_pos_base));
  return 1;
}

int take_snapshot(int zdok) {
  int cnt;

  /* trigger a snapshot capture */
  *ctrl_p[zdok] = 2;
  *ctrl_p[zdok] = 3;
  for(cnt = 0; *stat_p[zdok] & 0x80000000; cnt++) {
    if(cnt > 100) {
      CMD_STATUS = SNAP_TO;
      return 0;
    }
  }
  return *stat_p[zdok];	/* return length of snapshot */
}

/* Cores are read out in the sequence A, C, B, D , but should be in the natural
 * order in the ogp array. */
int startpos[] = {0, 2, 1, 3};
og_rtn og_from_noise(int len, signed char *snap) {
  og_rtn rtn;
  float avg, amp;
  int sum, i, cnt, core, code, start_i;

  memset((char *)&rtn, 0, sizeof(rtn));

  for(core = 0; core < 4; core++) {
    start_i = startpos[core];
    cnt = 0;
    sum = 0;
    amp = 0;
    for(i=start_i; i < len; i+= 4) {
      cnt++;
      code = snap[i];
      sum += code;
      if(code == -128 || code == 127) {
        rtn.overload_cnt[core]++;
      }
    }
    avg = (float)sum / cnt;
    for(i=start_i; i < len; i+= 4) {
      amp += fabs((float)snap[i] - avg);
    }
    avg *= (-500.0/256);
    rtn.avz += avg;
    rtn.avamp += amp/cnt;
    rtn.offs[core] = avg;
    rtn.gains[core] = amp/cnt;
  }
  rtn.avz /= 4;
  rtn.avamp /= 4;
  for(core = 0; core < 4; core++) {
    rtn.gains[core] = 100 * (rtn.avamp - rtn.gains[core])/rtn.avamp;
  }
  return(rtn);
}

/* Routine to take an adc snapshot once a second and keep
 * SWARM_LOADING_FACTOR_V2_F and SWARM_SAMPLER_HIST_V256_V2_L up to date */
void *monitor_adc(void *tr) {
  int status, len;
  int hist_cnt, n, val, sum, ssq;
  int zdok;
  int hist[2][256];
  double avg;
  float loading_factor[2];
#define NHIST 60

  bzero(&hist, sizeof(hist));
  hist_cnt = 0;
  while(run_adc_monitor == TRUE) {
    for(zdok = 0; zdok < 2; zdok++) {
      sum = 0;
      ssq = 0;
      pthread_mutex_lock(&snap_mutex);
      len = take_snapshot(zdok);
      pthread_mutex_unlock(&snap_mutex);
      for(n = 0; n < len; n++) {
        val = snap_p[zdok][n];
        sum += val;
        ssq += val*val;
        hist[zdok][val+128]++;
      }
      pthread_mutex_unlock(&snap_mutex);
      avg = (double)sum/len;
      loading_factor[zdok] =
           -20*log10f((float)(128/sqrt((double)ssq/len - avg*avg)));
    }
    status = dsm_write(DSM_MONITOR_CLASS, DSM_ADC_LOADING, loading_factor);
    if(++hist_cnt >= NHIST) {
      status |= dsm_write("obscon", DSM_ADC_HIST, hist);
      hist_cnt = 0;
      bzero(&hist, sizeof(hist));
    }
    if (status != DSM_SUCCESS) {
      CMD_STATUS = STRUCT_GET_ELEMENT;
    }
    sleep(1);
  }
  return tr;
}

int get_snapshot_cmd(int zdok){
  int i = 0;
  int len;
  FILE *fp;

  umask(0000);
  if((fp = fopen(SNAP_NAME, "w")) == NULL){
    CMD_STATUS = FILE_OPEN;
    return KATCP_RESULT_FAIL;
  }
  pthread_mutex_lock(&snap_mutex);
  len = take_snapshot(zdok);
  if(len == 0) {
    CMD_STATUS = SNAP_FAIL;
    fclose(fp);
    return KATCP_RESULT_FAIL;
  }
  for(i = 0; i < len; i++) {
    fprintf(fp, "%d\n", snap_p[zdok][i]);
  }
  adc_cmd_rtn[2] = len;
  pthread_mutex_unlock(&snap_mutex);
  fclose(fp);
  return KATCP_RESULT_OK;
}

int measure_og_cmd(int zdok, int rpt_arg){
  int rpt = 100;
  int i, n;
  int len;
  float offs[2][4], gains[2][4], oflow[2][4];
  float avz[2], avamp[2];
  og_rtn single_og;

  if(rpt_arg != 0) {
    rpt = rpt_arg;
    if(rpt <= 0 || rpt > 2000) {
      CMD_STATUS = BAD_RPT;
      return FAIL;
    }
  }
  i = read_adc_calibrations();
  if(i != OK) return(i);
  dsm_structure_get_element(&dsm_adc_cal, DEL_OFFS, offs); 
  dsm_structure_get_element(&dsm_adc_cal, DEL_GAINS, gains); 
  dsm_structure_get_element(&dsm_adc_cal, DEL_OVL_CNT , oflow); 
  dsm_structure_get_element(&dsm_adc_cal, DEL_AVZ, avz); 
  dsm_structure_get_element(&dsm_adc_cal, DEL_AVAMP, avamp); 

  /* Clear the values for the zdok we will measure */
  for(i = 0; i < 4; i++) {
    offs[zdok][i] = 0;
    gains[zdok][i] = 0;
    oflow[zdok][i] = 0;
  }
  avz[zdok] = 0;
  avamp[zdok] = 0;
  for(n = rpt; n > 0; n--) {
    pthread_mutex_lock(&snap_mutex);
    len = take_snapshot(zdok);
    if(len == 0) {
      CMD_STATUS = SNAP_FAIL;
      return FAIL;
    }
    single_og = og_from_noise(len, snap_p[zdok]);
    pthread_mutex_unlock(&snap_mutex);
    avz[zdok] += single_og.avz;
    avamp[zdok] += single_og.avamp;
    for(i = 0; i < 4; i++) {
      offs[zdok][i] += single_og.offs[i];
      gains[zdok][i] += single_og.gains[i];
      oflow[zdok][i] += single_og.overload_cnt[i];
    }
  }
  for(i = 0; i < 4; i++) {
    offs[zdok][i] /= rpt;
    gains[zdok][i] /= rpt;
  }
  avz[zdok] /= rpt;
  avamp[zdok] /= rpt;
  dsm_structure_set_element(&dsm_adc_cal, DEL_OFFS, offs); 
  dsm_structure_set_element(&dsm_adc_cal, DEL_GAINS, gains); 
  dsm_structure_set_element(&dsm_adc_cal, DEL_OVL_CNT , oflow); 
  dsm_structure_set_element(&dsm_adc_cal, DEL_AVZ, avz); 
  dsm_structure_set_element(&dsm_adc_cal, DEL_AVAMP, avamp); 
  return(write_adc_calibrations());
}

#if 0
int set_ogp_cmd(struct tbs_raw *tr, int zdok){
  float offs[2][4], gains[2][4], phases[2][4];

  if(set_spi_pointer(tr, zdok) == 0) {
    return KATCP_RESULT_FAIL;
  }
  read_adc_calibrations();
  dsm_structure_get_element(&dsm_adc_cal, DEL_OFFS, offs); 
  dsm_structure_get_element(&dsm_adc_cal, DEL_GAINS, gains); 
  dsm_structure_get_element(&dsm_adc_cal, DEL_GAINS, oflow); 
  set_ogp_registers(ogp);
  return KATCP_RESULT_OK;
}

/* Optional arguments zdok [0] */
int get_ogp_cmd(struct tbs_raw *tr, int zdok){
  float ogp[12];

  if(set_spi_pointer(tr, zdok) == 0) {
    return KATCP_RESULT_FAIL;
  }
  get_ogp_registers(ogp);
  print_ogp(d, ogp);
  return KATCP_RESULT_OK;
}

int update_ogp_cmd(struct tbs_raw *tr, int zdok){
  int i;
  struct tbs_raw *tr;
  float ogp_reg[12], ogp_meas[12];

  if(set_spi_pointer(tr, zdok) == 0) {
    return KATCP_RESULT_FAIL;
  }
  get_ogp_registers(ogp_reg);
  for(i = 0; i < 12; i++) {
    ogp_reg[i] += ogp_meas[i];
  }
  sprintf(fname, "%s%d", OGP_BASE_NAME, zdok);
  if(write_ogp_file(d, fname, ogp_reg) == 0) {
    return KATCP_RESULT_FAIL;
  }
  set_ogp_registers(ogp_reg);
  return KATCP_RESULT_OK;
}
#endif

void start_adc_monitor_cmd(struct tbs_raw *tr){
  int status;

  /* Set flag run adc monitoring of input level and histogram */
  run_adc_monitor = TRUE;

  /* Start the thread */
  status = pthread_create(&adc_monitor_thread,NULL, monitor_adc, (void *)tr);
  if (status < 0){
    CMD_STATUS = MONITOR_THREAD;
    run_adc_monitor = FALSE;
  }
}

/* int stop_adc_monitor_cmd(struct katcp_dispatch *d, int argc){ */
void stop_adc_monitor_cmd(void){

  /* Set flag to stop adc_monitor measuring and reporting */
  run_adc_monitor = FALSE;

  /* Wait until the thread stops */
  if(pthread_join(adc_monitor_thread, NULL)) {
    CMD_STATUS = MONITOR_NOT_JOINED;
  } else {
    CMD_STATUS = OK;
  }
}

/* SIGINT handler stops waiting */
void sigint_handler(int sig){
  run_cmd_monitor = FALSE;
}

/* cmd_thread routine to monitor SWARM_ADC_CMD_L */
void *cmd_monitor(void *tr) {
  char hostName[DSM_NAME_LENGTH];
  char varName[DSM_NAME_LENGTH];
  int cmd[3];
  int zdok, zdok_cmd;
  int rtn;
  static int zdokStart[3] = {0, 1, 0};
  static int zdokLimit[3] = {1, 2, 2};

  /* Set our INT signal handler so dsm_read_wait can be interrupted */
  signal(SIGINT, sigint_handler);

  while(run_cmd_monitor == TRUE) {
    bzero(adc_cmd_rtn, sizeof(adc_cmd_rtn));
    rtn = dsm_read_wait(hostName, varName, cmd);
    if(rtn == DSM_INTERRUPTED)
      return (void *)0;
    if(cmd[0] < START_ADC_MONITOR) {
      zdok_cmd = cmd[1];
      if(zdok_cmd < 0 || zdok_cmd > 2){
        CMD_STATUS = BAD_ZDOK;
        return (void *)0;
      }
      for(zdok = zdokStart[zdok_cmd]; zdok < zdokLimit[zdok_cmd]; zdok++) {
        switch(cmd[0]) {
	case TAKE_SNAPSHOT:
	  get_snapshot_cmd(zdok);
	  break;
	case SET_OGP:
	  break;
	case MEASURE_OG:
          rtn = measure_og_cmd(zdok, 0);
	  break;
	case UPDATE_OGP:
	  break;
        default:
          CMD_STATUS = UNK;
	}
	if(rtn != OK) {
	  adc_cmd_rtn[1] = zdok;
	  break;
	}
      }
    } else {
      switch(cmd[0]) {
      case START_ADC_MONITOR:
        start_adc_monitor_cmd((struct tbs_raw *)tr);
        break;
      case STOP_ADC_MONITOR:
        stop_adc_monitor_cmd();
        break;
      default:
        CMD_STATUS = UNK;
      }
    }
    sleep(1);
    dsm_write_notify(HAL, DSM_CMD_RTN, &adc_cmd_rtn);
  }
  return (void *)0;
}

int start_cmd_monitor_cmd(struct katcp_dispatch *d, int argc){
  int status;
  static struct tbs_raw *tr;
  /* Grab the mode pointer */
  if((tr = get_mode_pointer(d)) == NULL)
    return(KATCP_RESULT_FAIL);

  set_snapshot_pointers(tr);
  /* Set flag run adc monitoring of input level and histogram */
  run_cmd_monitor = TRUE;
  status = dsm_structure_init(&dsm_adc_cal, DSM_CAL_STRUCT );
  if (status != DSM_SUCCESS) {
    CMD_STATUS = STRUCT_INIT;
    return(KATCP_RESULT_FAIL);
  }
  if(dsm_monitor(HAL, DSM_CMD) != DSM_SUCCESS) {
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL,
                "Error setting up monitoring of dsm command.");
    return KATCP_RESULT_FAIL;
  }
  /* Start the thread */
  status = pthread_create(&cmd_monitor_thread,NULL, cmd_monitor, (void *)tr);
  if (status < 0){
    log_message_katcp(d, KATCP_LEVEL_ERROR, NULL,
                "could not create adc command monitor thread");
    run_cmd_monitor = FALSE;
    return KATCP_RESULT_FAIL;
  }
  return KATCP_RESULT_OK;
}

int stop_cmd_monitor_cmd(struct katcp_dispatch *d, int argc){

  if(run_adc_monitor == TRUE) {
    stop_adc_monitor_cmd();
  }
  if(run_cmd_monitor == FALSE) {
    return KATCP_RESULT_FAIL;
  }
  /* Set flag to stop cmd_monitor thread */
  pthread_kill(cmd_monitor_thread, SIGINT);

  /* Wait until the thread stops */
  pthread_join(cmd_monitor_thread, NULL);
  dsm_clear_monitor();
  dsm_structure_destroy(&dsm_adc_cal);

  return KATCP_RESULT_OK;
}


struct PLUGIN KATCP_PLUGIN = {
  .n_cmds = 2,
  .name = "sma_adc_dsm",
  .version = KATCP_PLUGIN_VERSION,
  .init = start_cmd_monitor_cmd,
  .uninit = stop_cmd_monitor_cmd,
  .cmd_array = {
#if 0
    {
      .name = "?get-snapshot", 
      .desc = "get a snapshot and store it in /instance/snap.  Optional argument zdok",
      .cmd = get_snapshot_cmd
    },
    {
      .name = "?measure-og", 
      .desc = "Take repeated snapshots and mesasure offset and gain assuming noise input.  Write result in /instance/ogp_meas.  Optional arguments zdoc (0) and rpt(30)",
      .cmd = measure_og_cmd
    },
    {
      .name = "?get-ogp", 
      .desc = "read ogp from the adc and write to /instance/ogp",
      .cmd = get_ogp_cmd
    },
    {
      .name = "?set-ogp", 
      .desc = "read ogp from a file and set the adc ogp registers.  Optional arguments zdok [0] and fname [/instance/ogp]",
      .cmd = set_ogp_cmd
    },
    {
      .name = "?update-ogp", 
      .desc = "read /instance/ogp_meas and add to the ogp registers in the adc.  Also store results in /instance/ogp.  Optional argument zdok",
      .cmd = update_ogp_cmd
    },
#endif
    {
      .name = "?start-cmd-thread", 
      .desc = "start the adc monitor thread",
      .cmd = start_cmd_monitor_cmd
    },
    {
      .name = "?stop-cmd-thread", 
      .desc = "stop the adc monitor thread",
      .cmd = stop_cmd_monitor_cmd
    },
  }
};

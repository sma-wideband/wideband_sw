#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <rpc/pmap_clnt.h>
#include <rpc/rpc.h>
#include <unistd.h>
#include <string.h>
#include <memory.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include "dDS.h"
#include "integration.h"

/* KATCP libraries */
#include "katcl.h"
#include "katcp.h"

#define OK     (0)
#define ERROR (-1)

#define SERVER_PRIORITY (10)

pthread_t serverTId;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
volatile sig_atomic_t package_received = 0;

/* Global package shared between threads */
intgWalshPackage *dds_package;

statusCheck(dDSStatus *status)
{
  if (status->status == DDS_SUCCESS)
    printf("O.K.\n");
  else
    printf("DDS server returned error status, reason = %d\n",
      status->reason);
}

void intgprog_1(struct svc_req *rqstp, register SVCXPRT *transp)
{
  union {
    INTG__test_struct intgconfigure_1_arg;
    intgParameters intgsetparams_1_arg;
    intgCommand intgcommand_1_arg;
    intgDumpFile intgnewfile_1_arg;
    attenuationRequest intgattenadjust_1_arg;
    intgWalshPackage intgsyncwalsh_1_arg;
    intgCommand intgreportconfiguration_1_arg;
    intgC2DCParameters intgc2dcrates_1_arg;
    intgCommand1IP intgcommand1ip_1_arg;
    intgChannelCounts intgsetchannels_1_arg;
  } argument;
  char *result;
  xdrproc_t _xdr_argument, _xdr_result;
  char *(*local)(char *, struct svc_req *);
  
  switch (rqstp->rq_proc) {
  case NULLPROC:
    (void) svc_sendreply (transp, (xdrproc_t) xdr_void, (char *)NULL);
    return;
    
  case INTGSYNCWALSH:
    _xdr_argument = (xdrproc_t) xdr_intgWalshPackage;
    _xdr_result = (xdrproc_t) xdr_intgWalshResponse;
    local = (char *(*)(char *, struct svc_req *)) intgsyncwalsh_1_svc;
    break;
  default:
    svcerr_noproc (transp);
    return;
  }
  memset ((char *)&argument, 0, sizeof (argument));
  if (!svc_getargs (transp, (xdrproc_t) _xdr_argument, (caddr_t) &argument)) {
    svcerr_decode (transp);
    return;
  }
  result = (*local)((char *)&argument, rqstp);
  if (result != NULL && !svc_sendreply(transp, (xdrproc_t) _xdr_result, result)) {
    svcerr_systemerr (transp);
  }
  if (!svc_freeargs (transp, (xdrproc_t) _xdr_argument, (caddr_t) &argument)) {
    fprintf (stderr, "%s", "unable to free arguments");
    exit (1);
  }
  return;
}

intgWalshResponse *result;
intgWalshResponse *intgsyncwalsh_1(intgWalshPackage *package, CLIENT *cl)
{
  int i, j, pattern_len, step_len;

  result = malloc(sizeof(intgWalshResponse));
  if (result == NULL) {
    perror("malloc of result structure for intgsyncwalsh_1\n");
    exit(ERROR);
  }

  if (!package_received) {
    pthread_mutex_lock(&mutex);
    dds_package = package;

    dds_package->pattern = package->pattern;
    pattern_len = dds_package->pattern.pattern_len;
    dds_package->pattern.pattern_val = malloc(pattern_len * sizeof(intgWalshPattern));
    for (i=0; i<pattern_len; i++) {
      dds_package->pattern.pattern_val[i] = package->pattern.pattern_val[i];
      dds_package->pattern.pattern_val[i].step = package->pattern.pattern_val[i].step;

      step_len = dds_package->pattern.pattern_val[i].step.step_len;
      if (step_len == 0) {
	dds_package->pattern.pattern_val[i].step.step_val = NULL;
      } else {
	dds_package->pattern.pattern_val[i].step.step_val = malloc(step_len * sizeof(int));
      }
      for (j=0; j<step_len; j++) {
	printf("%d:%d", i, j);
	dds_package->pattern.pattern_val[i].step.step_val[j] = package->pattern.pattern_val[i].step.step_val[j];
      }
    }

    pthread_mutex_unlock(&mutex);
    package_received = 1;
  }

  result->status = OK;
  return result;
}

int handle_package(intgWalshPackage *package, int hb_offset) {
  struct katcl_line *l;
  int i, nPatterns, step_len, total, katcp_result;
  struct timespec sowf_ts, armed_at;
  struct tm curr;
  struct tm sowf;
  time_t raw;

  /* Process the Walsh table */
  printf("Received a package from the DDS Server - contents:\n");
  nPatterns = package->pattern.pattern_len;
  /* printf("Number of Walsh Patterns %d, listed below\n", nPatterns); */
  /* for (i = 1; i < nPatterns; i++) { */
  /*   int j; */

  /*   step_len = package->pattern.pattern_val[i].step.step_len; */
  /*   printf("%2d: %d ", i, step_len); */
  /*   for (j = 0; j < step_len; j++) */
  /*     printf("%d", package->pattern.pattern_val[i].step.step_val[j]); */
  /*   printf("\n"); */
  /* } */

  /* Print a human-readable version of the agreed upon start of Walsh hearbeat */
  printf("Start at day %d of year %d\n", package->startDay, package->startYear);
  printf("at %02d:%02d:%02d.%06d\n", package->startHour, package->startMin,
	 package->startSec, package->startuSec);
  
  /* Get a struct tm of current time and replace with SOWF info */
  time(&raw);
  curr = *gmtime(&raw);
  sowf = curr;
  sowf.tm_year = package->startYear - 1900;
  sowf.tm_yday = package->startDay;
  sowf.tm_hour = package->startHour;
  sowf.tm_min = package->startMin;
  sowf.tm_sec = package->startSec;
  printf("Current time: %d\n", (int)mktime(&curr));
  printf("SOWF time:    %d\n", (int)mktime(&sowf));
  
  /* Convert it to a timespec */
  sowf_ts.tv_sec = mktime(&sowf);
  sowf_ts.tv_nsec = (package->startuSec) * 1e3;

  /* connect to roach2-02 */
  l = create_name_rpc_katcl("roach2-03");
  if(l == NULL){
    fprintf(stderr, "unable to create client connection to roach2-02: %s\n", strerror(errno));
    result->status = ERROR;
    return ERROR;
  }

  /* send the walsh arm request, with 60s timeout */
  katcp_result = send_rpc_katcl(l, 60000,
  				KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?sma-walsh-arm",
  				KATCP_FLAG_ULONG, sowf_ts.tv_sec,
  				KATCP_FLAG_ULONG, sowf_ts.tv_nsec,
  				KATCP_FLAG_LAST  | KATCP_FLAG_SLONG, (long)hb_offset,
  				NULL);

  clock_gettime(CLOCK_REALTIME, &armed_at);
  printf("Presumably armed at: %d.%06d\n", (int)armed_at.tv_sec, (int)armed_at.tv_nsec);

  /* result is 0 if the reply returns "ok", 1 if it failed and -1 if things went wrong doing IO or otherwise */
  printf("result of ?sma-walsh-arm is %d\n", katcp_result);

  /* you can examine the content of the reply with the following functions */
  total = arg_count_katcl(l);
  printf("have %d arguments in reply\n", total);
  for(i = 0; i < total; i++){
    /* for binary data use the arg_buffer_katcl, string will stop at the first occurrence of a \0 */
    printf("reply[%d] is <%s>\n", i, arg_string_katcl(l, i));
  }

  destroy_rpc_katcl(l);

  return OK;
}

intgWalshResponse *intgsyncwalsh_1_svc(intgWalshPackage *package,
				       struct svc_req *dummy)
{
  CLIENT *cl = NULL;
  
  result = intgsyncwalsh_1(package, cl);
  return result;
}

void *server(void *arg)
{
  printf("The server thread has started\n");
  register SVCXPRT *transp;
  
  pmap_unset(INTGPROG, INTGVERS);
  
  transp = svcudp_create(RPC_ANYSOCK);
  if (transp == NULL) {
    fprintf(stderr, "%s", "cannot create udp service.\n");
    exit(1);
  }
  if (!svc_register(transp, INTGPROG, INTGVERS, intgprog_1, IPPROTO_UDP)) {
    fprintf(stderr, "%s", "unable to register (INTGPROG, INTGVERS, udp).\n");
    exit(1);
  }
  
  transp = svctcp_create(RPC_ANYSOCK, 0, 0);
  if (transp == NULL) {
    fprintf(stderr, "%s", "cannot create tcp service.\n");
    exit(1);
  }
  if (!svc_register(transp, INTGPROG, INTGVERS, intgprog_1, IPPROTO_TCP)) {
    fprintf(stderr, "%s", "unable to register (INTGPROG, INTGVERS, tcp).\n");
    exit(1);
  }
  
  svc_run();
  fprintf(stderr, "%s", "svc_run returned\n");
  exit(1);
  /* NOTREACHED */
}

int main(int argc, char **argv)
{
  int status;
  int hb_offset = 0;
  char hostName[20];
  dDSCommand command;
  pthread_attr_t attr;
  CLIENT *cl;
  
  /* Create the RPC server thread */
  if (pthread_attr_init(&attr) != OK) {
    perror("pthread_attr_init attr");
    exit(ERROR);
  }
  if ((status = pthread_create(&serverTId, &attr, server,
			       (void *) 12)) != OK) {
    perror("pthread_create for the server thread");
    fprintf(stderr, "pthread_create returned status of %d\n", status);
    exit(ERROR);
  }

  /* Make the client call to the DDS server */
  if (!(cl = clnt_create("172.22.4.189", DDSPROG, DDSVERS, "tcp"))) {
    clnt_pcreateerror("Creating handle to server on newdds");
    exit(ERROR);
  }
  command.antenna = DDS_ALL_ANTENNAS;
  command.receiver = DDS_ALL_RECEIVERS;
  command.refFrequency = 9.0e6;
  command.command = DDS_START_WALSH;
  gethostname(command.client, 20);
  command.client[19] = (char)0;
  printf("This client's name is \"%s\".\n", command.client);
  statusCheck(ddsrequest_1(&command, cl));
  //pthread_join(serverTId, NULL);

  while (!package_received) {
    usleep(1);
  }

  if (argc == 2) {
    hb_offset = atoi(argv[1]);
  }

  handle_package(dds_package, hb_offset);
}

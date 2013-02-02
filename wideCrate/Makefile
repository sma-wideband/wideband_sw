IDIR=../../katcp_devel
RPC = /global/rpcFiles/
wideCrate: wideCrate.c dDS.h integration.h  Makefile
	gcc -o wideCrate -pthread wideCrate.c -I$(IDIR)/katcp dDS_clnt.o dDS_xdr.o \
	integration_xdr.o -lkatcp -lrt -L$(IDIR)/katcp

dDS.h: $(RPC)dDS.x Makefile
	cp $(RPC)dDS.x ./
	rpcgen dDS.x
	gcc -c dDS_clnt.c
	gcc -c dDS_xdr.c
	rm dDS_clnt.c dDS_svc.c dDS_xdr.c dDS.x

integration.h: $(RPC)integration.x Makefile
	cp $(RPC)integration.x ./
	rpcgen integration.x
	gcc -c integration_svc.c
	gcc -c integration_xdr.c
	rm integration_clnt.c integration_svc.c integration_xdr.c integration.x

clean:
	rm *.o wideCrate dDS.h integration.h

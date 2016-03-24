WORKSPACE_USER=cn
WORKSPACE_PATH=/home/${WORKSPACE_USER}/cnws

all: start-beagleboard-udp

start-roscore:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

start-bbb-proxy:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

start-beagleboard:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

start-beagleboard-udp:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@


clean:
	rm -f start-roscore
	rm -f start-bbb-proxy
	rm -f start-beagleboard
	rm -f start-beagleboard-udp

install: start-beagleboard-udp
	cp start-beagleboard-udp /usr/local/bin/
	cp beagleboard-udp.conf /etc/init/
	initctl reload-configuration

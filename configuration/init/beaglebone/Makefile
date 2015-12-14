WORKSPACE_USER=cn
WORKSPACE_PATH=/home/${WORKSPACE_USER}/cnws

all: start-roscore start-bbb-proxy start-beagleboard

start-roscore:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

start-bbb-proxy:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

start-beagleboard:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@


clean:
	rm -f start-roscore
	rm -f start-bbb-proxy
	rm -f start-beagleboard

install: start-roscore start-bbb-proxy start-beagleboard
	cp start-roscore start-bbb-proxy start-beagleboard /usr/local/bin/
	cp roscore.conf bbb-proxy.conf beagleboard.conf /etc/init/
	initctl reload-configuration

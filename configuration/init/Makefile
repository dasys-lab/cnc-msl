WORKSPACE_USER=cn
WORKSPACE_PATH=/home/${WORKSPACE_USER}/cnws

all: start-roscore start-udp-proxy

start-roscore:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

start-udp-proxy:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

clean:
	rm -f start-roscore start-udp-proxy

install: start-roscore start-udp-proxy
	cp start-roscore start-udp-proxy /usr/local/bin/
	cp *.conf /etc/init/
	initctl reload-configuration

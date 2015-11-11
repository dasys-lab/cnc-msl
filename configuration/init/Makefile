WORKSPACE_USER=cn
WORKSPACE_PATH=/home/${WORKSPACE_USER}/cnws

all: start-process-manager

start-process-manager:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@
clean:
	rm -f start-process-manager

install: start-process-manager
	cp start-roscore start-udp-proxy /usr/local/bin/
	cp process_manager.conf /etc/init/
	initctl reload-configuration

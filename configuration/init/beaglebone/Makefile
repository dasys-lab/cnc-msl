WORKSPACE_USER=cn
WORKSPACE_PATH=/home/${WORKSPACE_USER}/cnws

all: start-beagleboard-udp

start-beagleboard-udp:
	sed 's,USERTPL,${WORKSPACE_USER},g;s,WORKSPACETPL,${WORKSPACE_PATH},g' $@.tpl > $@ 
	chmod +x $@

clean:
	rm -f start-beagleboard-udp

install: start-beagleboard-udp
	cp start-beagleboard-udp /usr/local/bin/
	cp beagleboard-udp.service /lib/systemd/system/
	systemctl enable beagleboard-udp
	systemctl daemon-reload

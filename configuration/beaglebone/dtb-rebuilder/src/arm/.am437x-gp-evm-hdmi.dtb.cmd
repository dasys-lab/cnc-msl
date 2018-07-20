cmd_src/arm/am437x-gp-evm-hdmi.dtb = cpp -Wp,-MD,src/arm/.am437x-gp-evm-hdmi.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am437x-gp-evm-hdmi.dtb.dts.tmp src/arm/am437x-gp-evm-hdmi.dts ; dtc -O dtb -o src/arm/am437x-gp-evm-hdmi.dtb -b 0 -i src/arm -d src/arm/.am437x-gp-evm-hdmi.dtb.d.dtc.tmp src/arm/.am437x-gp-evm-hdmi.dtb.dts.tmp ; cat src/arm/.am437x-gp-evm-hdmi.dtb.d.pre.tmp src/arm/.am437x-gp-evm-hdmi.dtb.d.dtc.tmp > src/arm/.am437x-gp-evm-hdmi.dtb.d
am437x-gp-evm-hdmi.o: src/arm/am437x-gp-evm-hdmi.dts \
 src/arm/am437x-gp-evm.dts src/arm/am4372.dtsi \
 include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/interrupt-controller/arm-gic.h \
 include/dt-bindings/interrupt-controller/irq.h src/arm/skeleton.dtsi \
 include/dt-bindings/pinctrl/am43xx.h include/dt-bindings/pwm/pwm.h \
 include/dt-bindings/sound/sii9022-audio.h
src/arm/am437x-gp-evm-hdmi.dtb: src/arm/.am437x-gp-evm-hdmi.dtb.dts.tmp src/arm/am43xx-clocks.dtsi

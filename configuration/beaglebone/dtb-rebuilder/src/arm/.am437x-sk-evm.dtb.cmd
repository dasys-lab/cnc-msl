cmd_src/arm/am437x-sk-evm.dtb = cpp -Wp,-MD,src/arm/.am437x-sk-evm.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am437x-sk-evm.dtb.dts.tmp src/arm/am437x-sk-evm.dts ; dtc -O dtb -o src/arm/am437x-sk-evm.dtb -b 0 -i src/arm -d src/arm/.am437x-sk-evm.dtb.d.dtc.tmp src/arm/.am437x-sk-evm.dtb.dts.tmp ; cat src/arm/.am437x-sk-evm.dtb.d.pre.tmp src/arm/.am437x-sk-evm.dtb.d.dtc.tmp > src/arm/.am437x-sk-evm.dtb.d
am437x-sk-evm.o: src/arm/am437x-sk-evm.dts src/arm/am4372.dtsi \
 include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/interrupt-controller/arm-gic.h \
 include/dt-bindings/interrupt-controller/irq.h src/arm/skeleton.dtsi \
 include/dt-bindings/pinctrl/am43xx.h include/dt-bindings/pwm/pwm.h \
 include/dt-bindings/input/input.h
src/arm/am437x-sk-evm.dtb: src/arm/.am437x-sk-evm.dtb.dts.tmp src/arm/am43xx-clocks.dtsi

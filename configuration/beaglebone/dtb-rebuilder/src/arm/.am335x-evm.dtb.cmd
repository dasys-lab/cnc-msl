cmd_src/arm/am335x-evm.dtb = cpp -Wp,-MD,src/arm/.am335x-evm.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-evm.dtb.dts.tmp src/arm/am335x-evm.dts ; dtc -O dtb -o src/arm/am335x-evm.dtb -b 0 -i src/arm -d src/arm/.am335x-evm.dtb.d.dtc.tmp src/arm/.am335x-evm.dtb.dts.tmp ; cat src/arm/.am335x-evm.dtb.d.pre.tmp src/arm/.am335x-evm.dtb.d.dtc.tmp > src/arm/.am335x-evm.dtb.d
am335x-evm.o: src/arm/am335x-evm.dts src/arm/am33xx.dtsi \
 include/dt-bindings/gpio/gpio.h include/dt-bindings/pinctrl/am33xx.h \
 include/dt-bindings/pinctrl/omap.h src/arm/skeleton.dtsi \
 include/dt-bindings/interrupt-controller/irq.h src/arm/tps65910.dtsi
src/arm/am335x-evm.dtb: src/arm/.am335x-evm.dtb.dts.tmp src/arm/am33xx-clocks.dtsi

cmd_src/arm/am335x-nano.dtb = cpp -Wp,-MD,src/arm/.am335x-nano.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-nano.dtb.dts.tmp src/arm/am335x-nano.dts ; dtc -O dtb -o src/arm/am335x-nano.dtb -b 0 -i src/arm -d src/arm/.am335x-nano.dtb.d.dtc.tmp src/arm/.am335x-nano.dtb.dts.tmp ; cat src/arm/.am335x-nano.dtb.d.pre.tmp src/arm/.am335x-nano.dtb.d.dtc.tmp > src/arm/.am335x-nano.dtb.d
am335x-nano.o: src/arm/am335x-nano.dts src/arm/am33xx.dtsi \
 include/dt-bindings/gpio/gpio.h include/dt-bindings/pinctrl/am33xx.h \
 include/dt-bindings/pinctrl/omap.h src/arm/skeleton.dtsi \
 src/arm/tps65217.dtsi
src/arm/am335x-nano.dtb: src/arm/.am335x-nano.dtb.dts.tmp src/arm/am33xx-clocks.dtsi

cmd_src/arm/am335x-base0033.dtb = cpp -Wp,-MD,src/arm/.am335x-base0033.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-base0033.dtb.dts.tmp src/arm/am335x-base0033.dts ; dtc -O dtb -o src/arm/am335x-base0033.dtb -b 0 -i src/arm -d src/arm/.am335x-base0033.dtb.d.dtc.tmp src/arm/.am335x-base0033.dtb.dts.tmp ; cat src/arm/.am335x-base0033.dtb.d.pre.tmp src/arm/.am335x-base0033.dtb.d.dtc.tmp > src/arm/.am335x-base0033.dtb.d
am335x-base0033.o: src/arm/am335x-base0033.dts \
 src/arm/am335x-igep0033.dtsi src/arm/am33xx.dtsi \
 include/dt-bindings/gpio/gpio.h include/dt-bindings/pinctrl/am33xx.h \
 include/dt-bindings/pinctrl/omap.h src/arm/skeleton.dtsi \
 src/arm/tps65910.dtsi
src/arm/am335x-base0033.dtb: src/arm/.am335x-base0033.dtb.dts.tmp src/arm/am33xx-clocks.dtsi

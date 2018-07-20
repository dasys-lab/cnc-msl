cmd_src/arm/am335x-bone-cape-bone-robot.dtb = cpp -Wp,-MD,src/arm/.am335x-bone-cape-bone-robot.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-bone-cape-bone-robot.dtb.dts.tmp src/arm/am335x-bone-cape-bone-robot.dts ; dtc -O dtb -o src/arm/am335x-bone-cape-bone-robot.dtb -b 0 -i src/arm -d src/arm/.am335x-bone-cape-bone-robot.dtb.d.dtc.tmp src/arm/.am335x-bone-cape-bone-robot.dtb.dts.tmp ; cat src/arm/.am335x-bone-cape-bone-robot.dtb.d.pre.tmp src/arm/.am335x-bone-cape-bone-robot.dtb.d.dtc.tmp > src/arm/.am335x-bone-cape-bone-robot.dtb.d
am335x-bone-cape-bone-robot.o: src/arm/am335x-bone-cape-bone-robot.dts \
 src/arm/am33xx.dtsi include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h \
 src/arm/skeleton.dtsi src/arm/am335x-bone-common.dtsi \
 src/arm/am335x-bone-common-pinmux.dtsi src/arm/am335x-bone-robot.dtsi \
 src/arm/am335x-bone-ehrpwm1A.dtsi src/arm/am335x-bone-ehrpwm2A.dtsi \
 src/arm/am335x-bone-ehrpwm2B.dtsi
src/arm/am335x-bone-cape-bone-robot.dtb: src/arm/.am335x-bone-cape-bone-robot.dtb.dts.tmp src/arm/am33xx-clocks.dtsi src/arm/tps65217.dtsi

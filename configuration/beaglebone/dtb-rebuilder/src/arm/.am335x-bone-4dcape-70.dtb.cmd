cmd_src/arm/am335x-bone-4dcape-70.dtb = cpp -Wp,-MD,src/arm/.am335x-bone-4dcape-70.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-bone-4dcape-70.dtb.dts.tmp src/arm/am335x-bone-4dcape-70.dts ; dtc -O dtb -o src/arm/am335x-bone-4dcape-70.dtb -b 0 -i src/arm -d src/arm/.am335x-bone-4dcape-70.dtb.d.dtc.tmp src/arm/.am335x-bone-4dcape-70.dtb.dts.tmp ; cat src/arm/.am335x-bone-4dcape-70.dtb.d.pre.tmp src/arm/.am335x-bone-4dcape-70.dtb.d.dtc.tmp > src/arm/.am335x-bone-4dcape-70.dtb.d
am335x-bone-4dcape-70.o: src/arm/am335x-bone-4dcape-70.dts \
 src/arm/am33xx.dtsi include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h \
 src/arm/skeleton.dtsi src/arm/am335x-bone-common.dtsi \
 src/arm/am335x-bone-common-pinmux.dtsi \
 src/arm/am335x-bone-i2c2-cape-eeprom.dtsi \
 src/arm/am335x-bone-4dcape-70.dtsi src/arm/am335x-bone-led-gpio1-28.dtsi \
 src/arm/am335x-bone-keymap0.dtsi src/arm/am335x-bone-bl-gpio1-18.dtsi \
 src/arm/am335x-bone-panel-800x480.dtsi
src/arm/am335x-bone-4dcape-70.dtb: src/arm/.am335x-bone-4dcape-70.dtb.dts.tmp src/arm/am33xx-clocks.dtsi src/arm/tps65217.dtsi

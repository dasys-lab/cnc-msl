cmd_src/arm/am335x-bone-audio-reva.dtb = cpp -Wp,-MD,src/arm/.am335x-bone-audio-reva.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-bone-audio-reva.dtb.dts.tmp src/arm/am335x-bone-audio-reva.dts ; dtc -O dtb -o src/arm/am335x-bone-audio-reva.dtb -b 0 -i src/arm -d src/arm/.am335x-bone-audio-reva.dtb.d.dtc.tmp src/arm/.am335x-bone-audio-reva.dtb.dts.tmp ; cat src/arm/.am335x-bone-audio-reva.dtb.d.pre.tmp src/arm/.am335x-bone-audio-reva.dtb.d.dtc.tmp > src/arm/.am335x-bone-audio-reva.dtb.d
am335x-bone-audio-reva.o: src/arm/am335x-bone-audio-reva.dts \
 src/arm/am33xx.dtsi include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h \
 src/arm/skeleton.dtsi src/arm/am335x-bone-common.dtsi \
 src/arm/am335x-bone-common-pinmux.dtsi \
 src/arm/am335x-bone-i2c2-cape-eeprom.dtsi \
 src/arm/am335x-bone-audio-reva.dtsi
src/arm/am335x-bone-audio-reva.dtb: src/arm/.am335x-bone-audio-reva.dtb.dts.tmp src/arm/am33xx-clocks.dtsi src/arm/tps65217.dtsi

cmd_src/arm/am335x-boneblack-audio-revb.dtb = cpp -Wp,-MD,src/arm/.am335x-boneblack-audio-revb.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-boneblack-audio-revb.dtb.dts.tmp src/arm/am335x-boneblack-audio-revb.dts ; dtc -O dtb -o src/arm/am335x-boneblack-audio-revb.dtb -b 0 -i src/arm -d src/arm/.am335x-boneblack-audio-revb.dtb.d.dtc.tmp src/arm/.am335x-boneblack-audio-revb.dtb.dts.tmp ; cat src/arm/.am335x-boneblack-audio-revb.dtb.d.pre.tmp src/arm/.am335x-boneblack-audio-revb.dtb.d.dtc.tmp > src/arm/.am335x-boneblack-audio-revb.dtb.d
am335x-boneblack-audio-revb.o: src/arm/am335x-boneblack-audio-revb.dts \
 src/arm/am33xx.dtsi include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h \
 src/arm/skeleton.dtsi src/arm/am335x-bone-common.dtsi \
 src/arm/am335x-bone-common-pinmux.dtsi \
 src/arm/am335x-boneblack-emmc.dtsi \
 src/arm/am335x-bone-i2c2-cape-eeprom.dtsi \
 src/arm/am335x-bone-audio-revb.dtsi \
 src/arm/am335x-boneblack-nxp-hdmi-no-audio.dtsi
src/arm/am335x-boneblack-audio-revb.dtb: src/arm/.am335x-boneblack-audio-revb.dtb.dts.tmp src/arm/am33xx-clocks.dtsi src/arm/tps65217.dtsi

cmd_src/arm/am335x-boneblack-lcd3-01-00a2.dtb = cpp -Wp,-MD,src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.dts.tmp src/arm/am335x-boneblack-lcd3-01-00a2.dts ; dtc -O dtb -o src/arm/am335x-boneblack-lcd3-01-00a2.dtb -b 0 -i src/arm -d src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.d.dtc.tmp src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.dts.tmp ; cat src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.d.pre.tmp src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.d.dtc.tmp > src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.d
am335x-boneblack-lcd3-01-00a2.o: \
 src/arm/am335x-boneblack-lcd3-01-00a2.dts src/arm/am33xx.dtsi \
 include/dt-bindings/gpio/gpio.h include/dt-bindings/pinctrl/am33xx.h \
 include/dt-bindings/pinctrl/omap.h src/arm/skeleton.dtsi \
 src/arm/am335x-bone-common.dtsi src/arm/am335x-bone-common-pinmux.dtsi \
 src/arm/am335x-boneblack-emmc.dtsi \
 src/arm/am335x-bone-i2c2-cape-eeprom.dtsi \
 src/arm/am335x-bone-lcd3-01-00a2.dtsi \
 src/arm/am335x-bone-led-gpio3-19.dtsi src/arm/am335x-bone-keymap0.dtsi \
 src/arm/am335x-bone-bl-gpio1-18.dtsi \
 src/arm/am335x-bone-ti-tscadc-4-wire.dtsi \
 src/arm/am335x-bone-panel-320x240.dtsi
src/arm/am335x-boneblack-lcd3-01-00a2.dtb: src/arm/.am335x-boneblack-lcd3-01-00a2.dtb.dts.tmp src/arm/am33xx-clocks.dtsi src/arm/tps65217.dtsi

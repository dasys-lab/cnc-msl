cmd_src/arm/am335x-boneblack-bbb-exp-c.dtb = cpp -Wp,-MD,src/arm/.am335x-boneblack-bbb-exp-c.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am335x-boneblack-bbb-exp-c.dtb.dts.tmp src/arm/am335x-boneblack-bbb-exp-c.dts ; dtc -O dtb -o src/arm/am335x-boneblack-bbb-exp-c.dtb -b 0 -i src/arm -d src/arm/.am335x-boneblack-bbb-exp-c.dtb.d.dtc.tmp src/arm/.am335x-boneblack-bbb-exp-c.dtb.dts.tmp ; cat src/arm/.am335x-boneblack-bbb-exp-c.dtb.d.pre.tmp src/arm/.am335x-boneblack-bbb-exp-c.dtb.d.dtc.tmp > src/arm/.am335x-boneblack-bbb-exp-c.dtb.d
am335x-boneblack-bbb-exp-c.o: src/arm/am335x-boneblack-bbb-exp-c.dts \
 src/arm/am33xx.dtsi include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/pinctrl/am33xx.h include/dt-bindings/pinctrl/omap.h \
 src/arm/skeleton.dtsi src/arm/am335x-bone-common.dtsi \
 src/arm/am335x-bone-common-pinmux.dtsi \
 src/arm/am335x-boneblack-emmc.dtsi \
 src/arm/am335x-bone-i2c2-cape-eeprom.dtsi \
 src/arm/am335x-bone-bbb-exp-c.dtsi src/arm/am335x-bone-can0.dtsi \
 src/arm/am335x-can0.dtsi src/arm/am335x-bone-ttyO1.dtsi \
 src/arm/am335x-ttyO1.dtsi src/arm/am335x-bone-ttyO2.dtsi \
 src/arm/am335x-ttyO2.dtsi src/arm/am335x-bone-ttyO4.dtsi \
 src/arm/am335x-ttyO4.dtsi src/arm/am335x-bone-led-gpio2-4-gpio2-5.dtsi \
 src/arm/am335x-bone-keymap3.dtsi src/arm/am335x-bone-bl-gpio1-18.dtsi \
 src/arm/am335x-bone-panel-1024x600-24bit.dtsi
src/arm/am335x-boneblack-bbb-exp-c.dtb: src/arm/.am335x-boneblack-bbb-exp-c.dtb.dts.tmp src/arm/am33xx-clocks.dtsi src/arm/tps65217.dtsi

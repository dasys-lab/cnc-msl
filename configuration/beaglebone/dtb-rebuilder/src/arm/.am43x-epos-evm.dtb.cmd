cmd_src/arm/am43x-epos-evm.dtb = cpp -Wp,-MD,src/arm/.am43x-epos-evm.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.am43x-epos-evm.dtb.dts.tmp src/arm/am43x-epos-evm.dts ; dtc -O dtb -o src/arm/am43x-epos-evm.dtb -b 0 -i src/arm -d src/arm/.am43x-epos-evm.dtb.d.dtc.tmp src/arm/.am43x-epos-evm.dtb.dts.tmp ; cat src/arm/.am43x-epos-evm.dtb.d.pre.tmp src/arm/.am43x-epos-evm.dtb.d.dtc.tmp > src/arm/.am43x-epos-evm.dtb.d
am43x-epos-evm.o: src/arm/am43x-epos-evm.dts src/arm/am4372.dtsi \
 include/dt-bindings/gpio/gpio.h \
 include/dt-bindings/interrupt-controller/arm-gic.h \
 include/dt-bindings/interrupt-controller/irq.h src/arm/skeleton.dtsi \
 include/dt-bindings/pinctrl/am43xx.h include/dt-bindings/pwm/pwm.h \
 include/dt-bindings/sound/tlv320aic31xx-micbias.h
src/arm/am43x-epos-evm.dtb: src/arm/.am43x-epos-evm.dtb.dts.tmp src/arm/am43xx-clocks.dtsi

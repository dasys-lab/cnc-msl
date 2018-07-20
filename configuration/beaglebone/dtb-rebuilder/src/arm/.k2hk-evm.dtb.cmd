cmd_src/arm/k2hk-evm.dtb = cpp -Wp,-MD,src/arm/.k2hk-evm.dtb.d.pre.tmp -nostdinc -Iinclude -Isrc/arm -Itestcase-data -undef -D__DTS__ -x assembler-with-cpp -o src/arm/.k2hk-evm.dtb.dts.tmp src/arm/k2hk-evm.dts ; dtc -O dtb -o src/arm/k2hk-evm.dtb -b 0 -i src/arm -d src/arm/.k2hk-evm.dtb.d.dtc.tmp src/arm/.k2hk-evm.dtb.dts.tmp ; cat src/arm/.k2hk-evm.dtb.d.pre.tmp src/arm/.k2hk-evm.dtb.d.dtc.tmp > src/arm/.k2hk-evm.dtb.d
k2hk-evm.o: src/arm/k2hk-evm.dts src/arm/keystone.dtsi \
 include/dt-bindings/interrupt-controller/arm-gic.h \
 include/dt-bindings/interrupt-controller/irq.h src/arm/skeleton.dtsi
src/arm/k2hk-evm.dtb: src/arm/.k2hk-evm.dtb.dts.tmp src/arm/keystone-clocks.dtsi

.PHONY: all src clean distclean

all: src

src:
	@$(MAKE) -C src all
	#@cp -d lib/* ../../../../lib

clean:
	@$(MAKE) -C src clean

distclean:
	@$(MAKE) -C src distclean
	rm -rf ../lib/*

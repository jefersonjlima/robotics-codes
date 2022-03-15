# This file is only used for development for convinience functions as
# quick builds and tests

PROJECT=${shell pwd}
BIN=${PROJECT}/build/bin

.PHONY: build

build:
	mkdir -p build
	cd build && cmake -DCOMPILE_ALL=YES .. 			\
		&& make -j$(nproc)

clean:
	rm -rf build




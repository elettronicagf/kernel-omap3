tar xzvf binaries/ti-compat-wireless-wl12xx-2011-05-17-r3-m1-rc2.tgz
export KLIB_BUILD=..
export KLIB=./binaries/
cd compat-wireless-2.6
make clean
make -j2
make install-modules


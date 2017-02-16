#!/bin/bash
set -ex
pushd .
cd /tmp
git clone https://github.com/dermesser/libsocket
cd libsocket
mkdir build && cd build
cmake .. && make && sudo make install
popd

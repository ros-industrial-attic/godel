#!/bin/sh
set -ex
git clone https://github.com/dermesser/libsocket
cd libsocket
mkdir build && cd build
cmake .. && make && sudo make install

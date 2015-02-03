Godel Polygon Offset
==============

This package offsets polygon boundaries for use in Godel blending.

# Dependencies #
**openvoronoi**

Note: debian for openvoronoi is outdated, do not use.

From source:

- Install dependences
  - cmake
  - libqd-dev             http://crd.lbl.gov/~dhbailey/mpdist/
  - Boost graph library

- Download and build

  - cd ~/Downloads
  - git clone git://github.com/aewallin/openvoronoi.git
  - cd openvoronoi
  - mkdir bld
  - cd bld
  - cmake ../src
  - make CFLAGS="-DNDEBUG"
  - sudo make install

After installation, patch offset_sorter.hpp

- roscd godel_polygon_offset
- sudo patch /usr/local/include/openvoronoi/offset_sorter.hpp < src/offset_sorter.patch

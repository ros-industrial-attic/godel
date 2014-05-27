Godel Polygon Offset
==============

This package offsets polygon boundaries for use in Godel blending.

# Dependencies #
**openvoronoi**

Note: debian for openvoronoi is outdated, do not use.

It can be built from source:

- cd ~/Downloads
- git clone git://github.com/aewallin/openvoronoi.git
- cd openvoronoi
- mkdir bld
- cd bld
- cmake ../src
- make
- sudo make install

After installation, patch offset_sorter.hpp

- roscd godel_polygon_offset
- sudo patch /usr/local/include/openvoronoi/offset_sorter.hpp < src/offset_sorter.patch

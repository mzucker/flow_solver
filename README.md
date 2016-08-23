flow_solver
===========

Fast, automated solution of Flow Free puzzles in a single C file.
Full writeup coming soon to <https://mzucker.github.io/>

Building
========

No dependencies to install, just do this:

    cd /path/to/flow_solver
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

Using the software
==================

From the `build` directory, try:

    ./flow_solver ../puzzles/jumbo_14x14_01.txt

Puzzle file format
==================

Plain-ASCII file, one line per row of puzzle. Use the following
characters to specify color:

| Letter | Color        |
|--------|--------------|
| R      | red          |
| B      | blue         |
| Y      | yellow       |
| G      | green        |
| O      | orange       |
| C      | cyan         |
| M      | magenta      |
| m      | maroon       |
| P      | purple       |
| A      | gray         |
| W      | white        |
| g      | bright green |
| T      | tan          |
| b      | dark blue    |
| c      | dark cyan    |
| p      | pink         |

Any non-alphabetical, non-newline character becomes a blank space.
Puzzle size is set by the number of characters before the first
newline.




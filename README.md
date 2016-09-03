flow_solver
===========

Fast automated solver for Flow Free in a single C or Python file. 

Full writeup of C version at <https://mzucker.github.io/2016/08/28/flow-solver.html>.

Python writeup at <https://mzucker.github.io/2016/09/02/eating-sat-flavored-crow.html>.

Building the C version
======================

No dependencies to install, just do this:

    cd /path/to/flow_solver
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

Using the C version:
====================

From the `build` directory, try:

    ./flow_solver ../puzzles/jumbo_14x14_01.txt
    
Using the Python version:
=========================

Make sure you have [pycosat](https://github.com/ContinuumIO/pycosat) installed. If not, try:

    pip install pycosat
    
Then, from the project root directory, try:

    ./pyflowsolver.py puzzles/jumbo_14x14_01.txt

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




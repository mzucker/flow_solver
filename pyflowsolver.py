#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Module to solve Flow Free puzzles by reducing to SAT and invoking
pycosat's solver. The reduction to SAT does not automatically prevent
cycles from appearing which are disconnected from the pre-colored
cells; however, they are detected, and the solver will be run until
all cycles are eliminated.

'''

import sys
import operator
import itertools
from datetime import datetime
from argparse import ArgumentParser
import pycosat


LEFT = 1
RIGHT = 2
TOP = 4
BOTTOM = 8

DELTAS = [(LEFT, 0, -1),
          (RIGHT, 0, 1),
          (TOP, -1, 0),
          (BOTTOM, 1, 0)]

LR = LEFT | RIGHT
TB = TOP | BOTTOM
TL = TOP | LEFT
TR = TOP | RIGHT
BL = BOTTOM | LEFT
BR = BOTTOM | RIGHT

DIR_CODES = [LR, TB, TL, TR, BL, BR]

DIR_FLIP = {
    LEFT: RIGHT,
    RIGHT: LEFT,
    TOP: BOTTOM,
    BOTTOM: TOP
    }

ANSI_LOOKUP = dict(R=101, B=104, Y=103, G=42,
                   O=43, C=106, M=105, m=41,
                   P=45, A=100, W=107, g=102,
                   T=47, b=44, c=46, p=35)

DIR_LOOKUP = {
    LR: '─',
    TB: '│',
    TL: '┘',
    TR: '└',
    BL: '┐',
    BR: '┌'
    }

QUIET = False

######################################################################

def allpairs(collection):
    '''Return all combinations of two items from a collection, useful for
making a large number of SAT variables mutually exclusive.

    '''

    return itertools.combinations(collection, 2)

######################################################################

def no_two(satvars):
    '''Given a collection of SAT variables, generates clauses specifying
that no two of them can be true at the same time.

    '''

    return ((-a, -b) for (a, b) in allpairs(satvars))

######################################################################

def explode(puzzle):
    '''Iterator helper function to allow looping over 2D arrays without
nested 'for' loops.

    '''

    for i, row in enumerate(puzzle):
        for j, char in enumerate(row):
            yield i, j, char

######################################################################

def valid_pos(size, i, j):
    '''Check whether a position on a square grid is valid.'''
    return i >= 0 and i < size and j >= 0 and j < size

######################################################################

def all_neighbors(i, j):
    '''Return all neighbors of a grid square at row i, column j.'''
    return ((dir_bit, i+delta_i, j+delta_j)
            for (dir_bit, delta_i, delta_j) in DELTAS)

######################################################################

def valid_neighbors(size, i, j):
    '''Return all actual on-grid neighbors of a grid square at row i,
column j.'''

    return ((dir_bit, ni, nj) for (dir_bit, ni, nj)
            in all_neighbors(i, j)
            if valid_pos(size, ni, nj))

######################################################################

def parse_puzzle(filename):

    '''Read in the given file and parse it into a square array of strings,
as well as a dictionary which maps input characters to color
indices.

    '''

    try:
        with open(filename, 'r') as infile:
            puzzle = infile.read().splitlines()
    except IOError:
        print '{}: error opening file'.format(filename)
        return None, None

    size = len(puzzle[0])
    if len(puzzle) < size:
        print '{}:{} unexpected EOF'.format(filename, size)
        return None, None

    puzzle = puzzle[:size]

    colors = dict()
    color_count = []
    size = len(puzzle)

    for i, row in enumerate(puzzle):
        if len(row) != size:
            print '{}:{} row size mismatch'.format(filename, i+1)
            return None, None
        for j, char in enumerate(row):
            if char.isalnum():
                if colors.has_key(char):
                    color = colors[char]
                    if color_count[color]:
                        print '{}:{}:{} too many {} already'.format(
                            filename, i+1, j, char)
                        return None, None
                    color_count[color] = 1
                else:
                    color = len(colors)
                    colors[char] = color
                    color_count.append(0)

    for char, color in colors.iteritems():
        if not color_count[color]:
            print 'color {} has start but no end!'.format(char)
            return None, None

    if not QUIET:
        print 'read {}x{} puzzle with {} colors from {}'.format(
            size, size, len(colors), filename)
        print

    return puzzle, colors

######################################################################

def make_color_clauses(puzzle, colors, color_var):

    '''Generate CNF clauses entailing the N*M color SAT variables, where N
is the number of cells and M is the number of colors. Each cell
encodes a single color in a one-hot fashion.

    '''

    clauses = []
    num_colors = len(colors)
    size = len(puzzle)

    # for each cell
    for i, j, char in explode(puzzle):

        # check if solved already
        if char.isalnum():

            solved_color = colors[char]

            # color in this cell is this one
            clauses.append([color_var(i, j, solved_color)])

            # color in this cell is not the other ones
            for other_color in range(num_colors):
                if other_color != solved_color:
                    clauses.append([-color_var(i, j, other_color)])

            # gather neighbors' variables for this color
            neighbor_vars = [color_var(ni, nj, solved_color) for
                             _, ni, nj in valid_neighbors(size, i, j)]

            # one neighbor has this color
            clauses.append(neighbor_vars)

            # no two neighbors have this color
            clauses.extend(no_two(neighbor_vars))

        else:

            # one of the colors in this cell is set
            clauses.append([color_var(i, j, color)
                            for color in range(num_colors)])

            # no two of the colors in this cell are set
            cell_color_vars = (color_var(i, j, color) for
                               color in range(num_colors))

            clauses.extend(no_two(cell_color_vars))

    return clauses

######################################################################

def make_dir_vars(puzzle, start_var):

    '''Creates the direction-type SAT variables for each cell.'''

    size = len(puzzle)
    dir_vars = dict()
    num_dir_vars = 0

    for i, j, char in explode(puzzle):

        if char.isalnum():
            continue

        neighbor_bits = (dir_bit for (dir_bit, ni, nj)
                         in valid_neighbors(size, i, j))

        cell_flags = reduce(operator.or_, neighbor_bits, 0)

        dir_vars[i, j] = dict()

        for code in DIR_CODES:
            if cell_flags & code == code:
                num_dir_vars += 1
                dir_vars[i, j][code] = start_var + num_dir_vars

    return dir_vars, num_dir_vars

######################################################################

def make_dir_clauses(puzzle, colors, color_var, dir_vars):

    '''Generate clauses involving the color and direction-type SAT
variables. Each free cell must be exactly one direction, and
directions imply color matching with neighbors.

    '''

    dir_clauses = []
    num_colors = len(colors)
    size = len(puzzle)

    for i, j, char in explode(puzzle):

        if char.isalnum():
            continue

        cell_dir_dict = dir_vars[(i, j)]
        cell_dir_vars = cell_dir_dict.values()

        dir_clauses.append(cell_dir_vars)

        for dir_a, dir_b in allpairs(cell_dir_vars):
            dir_clauses.append([-dir_a, -dir_b])

        for dir_code, dir_var in cell_dir_dict.iteritems():

            for dir_bit, n_i, n_j in all_neighbors(i, j):

                for color in range(num_colors):
                    color_1 = color_var(i, j, color)
                    color_2 = color_var(n_i, n_j, color)
                    if dir_code & dir_bit:
                        dir_clauses.append([-dir_var, -color_1, color_2])
                        dir_clauses.append([-dir_var, color_1, -color_2])
                    elif valid_pos(size, n_i, n_j):
                        dir_clauses.append([-dir_var, -color_1, -color_2])

    return dir_clauses

######################################################################

def reduce_to_sat(puzzle, colors):

    '''Reduces the given puzzle to a SAT problem specified in CNF. Returns
a list of clauses where each clause is a list of single SAT variables,
possibly negated.

    '''

    size = len(puzzle)
    num_colors = len(colors)

    num_cells = size**2
    num_color_vars = num_colors * num_cells

    def color_var(i, j, color):
        '''Return the index of the SAT variable for the given color in row i,
 column j.

        '''
        return (i*size + j)*num_colors + color + 1

    start = datetime.now()

    color_clauses = make_color_clauses(puzzle,
                                       colors,
                                       color_var)

    dir_vars, num_dir_vars = make_dir_vars(puzzle, num_color_vars)

    dir_clauses = make_dir_clauses(puzzle, colors,
                                   color_var, dir_vars)

    num_vars = num_color_vars + num_dir_vars
    clauses = color_clauses + dir_clauses

    reduce_time = (datetime.now() - start).total_seconds()

    if not QUIET:
        print 'generated {:,} clauses over {:,} color variables'.format(
            len(color_clauses), num_color_vars, grouping=True)

        print 'generated {:,} dir clauses over {:,} dir variables'.format(
            len(dir_clauses), num_dir_vars)

        print 'total {:,} clauses over {:,} variables'.format(
            len(clauses), num_vars)

        print 'reduced to SAT in {:.3f} seconds'.format(reduce_time)
        print

    return color_var, dir_vars, clauses, reduce_time

######################################################################

def decode_solution(puzzle, colors, color_var, dir_vars, sol):

    '''Takes the solution set from SAT and decodes it by undoing the
one-hot encoding in each cell for color and direction-type. Returns a
2D array of (color, direction-type) pairs.

    '''

    sol = set(sol)
    num_colors = len(colors)

    decoded = []

    # fill each cell with color/dircode
    for i, row in enumerate(puzzle):

        decoded_row = []

        for j, char in enumerate(row):

            cell_color = -1

            for color in range(num_colors):
                if color_var(i, j, color) in sol:
                    assert cell_color == -1
                    cell_color = color

            assert cell_color != -1

            cell_dir_code = -1

            if not char.isalnum():

                for dir_code, dir_var in dir_vars[i, j].iteritems():
                    if dir_var in sol:
                        assert cell_dir_code == -1
                        cell_dir_code = dir_code

                assert cell_dir_code != -1

            decoded_row.append((cell_color, cell_dir_code))

        decoded.append(decoded_row)

    return decoded

######################################################################

def follow_path(decoded, visited, cur_i, cur_j):

    '''Follow a path starting from an arbitrary row, column location on
the grid until a non-path cell is detected, or a cycle is
found. Returns a list of (row, column) pairs on the path, as well as a
boolean flag indicating if a cycle was detected.

    '''

    prev_i, prev_j = -1, -1

    size = len(decoded)

    run = []
    is_cycle = False

    while True:
        advanced = False
        color, dir_code = decoded[cur_i][cur_j]
        visited[cur_i][cur_j] = 1
        run.append((cur_i, cur_j))

        for dir_bit, n_i, n_j in valid_neighbors(size, cur_i, cur_j):

            if (n_i, n_j) == (prev_i, prev_j):
                continue

            n_color, n_dir_code = decoded[n_i][n_j]

            if ((dir_code >= 0 and (dir_code & dir_bit)) or
                    (dir_code == -1 and n_dir_code >= 0 and
                     n_dir_code & DIR_FLIP[dir_bit])):

                assert color == n_color

                if visited[n_i][n_j]:
                    is_cycle = True
                else:
                    prev_i, prev_j = cur_i, cur_j
                    cur_i, cur_j = n_i, n_j
                    advanced = True
                break

        if not advanced:
            break

    return run, is_cycle

######################################################################

def detect_cycles(decoded, dir_vars):

    '''Examine the decoded SAT solution to see if any cycles exist; if so,
return the CNF clauses that need to be added to the problem in order
to prevent them.

    '''

    size = len(decoded)
    colors_seen = set()
    visited = [[0]*size for _ in range(size)]

    for i, j, (color, dir_code) in explode(decoded):
        if dir_code == -1:
            if color not in colors_seen:
                assert not visited[i][j]
                colors_seen.add(color)
                run, is_cycle = follow_path(decoded, visited, i, j)
                assert not is_cycle

    extra_clauses = []

    for i, j in itertools.product(range(size), range(size)):
        if not visited[i][j]:

            run, is_cycle = follow_path(decoded, visited, i, j)
            assert is_cycle

            clause = []

            for r_i, r_j in run:
                _, dir_code = decoded[r_i][r_j]
                dir_var = dir_vars[r_i, r_j][dir_code]
                clause.append(-dir_var)

            extra_clauses.append(clause)

    return extra_clauses

######################################################################

def show_solution(colors, decoded):

    '''Print the puzzle solution to the terminal.'''

    color_chars = [None]*len(colors)

    for char, color in colors.iteritems():
        color_chars[color] = char

    ansi_reset = '\033[0m'

    for decoded_row in decoded:
        for (color, dir_code) in decoded_row:

            assert color >= 0 and color < len(colors)

            if dir_code == -1:
                display_char = 'O'
            else:
                display_char = DIR_LOOKUP[dir_code]

            color_char = color_chars[color]

            if ANSI_LOOKUP.has_key(color_char):
                ansi_code = '\033[30;{}m'.format(ANSI_LOOKUP[color_char])
            else:
                ansi_code = ansi_reset

            sys.stdout.write(ansi_code + display_char)

        sys.stdout.write(ansi_reset + '\n')

######################################################################

def solve_sat(puzzle, colors, color_var, dir_vars, clauses):

    '''Solve the SAT now that it has been reduced to a list of clauses in
CNF.  This is an iterative process: first we try to solve a SAT, then
we detect cycles. If cycles are found, they are prevented from
recurring, and the next iteration begins. Returns the SAT solution
set, the decoded puzzle solution, and the number of cycle repairs
needed.

    '''

    start = datetime.now()

    decoded = None
    repairs = 0

    while True:

        sol = pycosat.solve(clauses) # pylint: disable=E1101

        if not isinstance(sol, list):
            break

        decoded = decode_solution(puzzle, colors, color_var, dir_vars, sol)

        extra_clauses = detect_cycles(decoded, dir_vars)

        if not extra_clauses:
            break

        clauses += extra_clauses
        repairs += 1

    solve_time = (datetime.now() - start).total_seconds()

    if not QUIET:
        if decoded is None:
            print 'solver returned {} after {:,} cycle '\
                'repairs and {:.3f} seconds'.format(
                    str(sol), repairs, solve_time)

        else:
            print 'obtained solution after {:,} cycle repairs '\
                'and {:.3f} seconds:'.format(
                    repairs, solve_time)
            print
            show_solution(colors, decoded)
            print

    return sol, decoded, repairs, solve_time

######################################################################

def pyflow_solver_main():

    '''Main loop if module run as script.'''

    parser = ArgumentParser(
        description='Solve Flow Free puzzles via reduction to SAT')

    parser.add_argument('filenames', metavar='PUZZLE', nargs='+',
                        help='puzzle file to load')

    parser.add_argument('-q', dest='quiet', default=False,
                        action='store_true',
                        help='quiet mode (reduce output)')

    args = parser.parse_args()

    global QUIET
    QUIET = args.quiet

    first = True

    max_width = max(len(f) for f in args.filenames)

    for filename in args.filenames:

        if not QUIET and not first:
            print '\n'+('*'*70)+'\n'

        puzzle, colors = parse_puzzle(filename)

        if colors is None:
            continue

        color_var, dir_vars, clauses, reduce_time = reduce_to_sat(puzzle, colors)

        sol, _, repairs, solve_time = solve_sat(puzzle, colors, color_var,
                                                dir_vars, clauses)

        total_time = reduce_time + solve_time

        if not QUIET:
            print 'finished in total of {:.3f} seconds'.format(
                total_time)
        else:
            if isinstance(sol, list):
                result_char = 's'
            elif str(sol) == 'UNSAT':
                result_char = 'u'
            else:
                result_char = 'f'

            print '{:>{}s} {} {:3d} {:12,.3f} {:12,.3f} {:12,.3f}'.format(
                filename, max_width, result_char, repairs,
                reduce_time, solve_time, total_time)

        first = False

######################################################################

if __name__ == '__main__':

    pyflow_solver_main()

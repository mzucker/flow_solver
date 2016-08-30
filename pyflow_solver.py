#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import pycosat
import operator
import itertools

from datetime import datetime


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

######################################################################

def allpairs(collection):
    return itertools.combinations(collection, 2)

######################################################################

def no_two(varlist):
    return ((-a, -b) for (a, b) in allpairs(varlist))

######################################################################

def explode(puzzle):
    for i, row in enumerate(puzzle):
        for j, char in enumerate(row):
            yield i, j, char

######################################################################

def valid_pos(size, i, j):
    return i >= 0 and i < size and j >= 0 and j < size

######################################################################

def all_neighbors(i, j):
    return ((dir_bit, i+delta_i, j+delta_j)
            for (dir_bit, delta_i, delta_j) in DELTAS)

def valid_neighbors(size, i, j):
    return ((dir_bit, ni, nj) for (dir_bit, ni, nj)
            in all_neighbors(i, j)
            if valid_pos(size, ni, nj))

######################################################################

def make_colors(filename, puzzle):

    colors = dict()
    color_count = []
    size = len(puzzle)

    for i, row in enumerate(puzzle):
        if len(row) != size:
            print '{}:{} row size mismatch'.format(filename, i+1)
            return None
        for j, char in enumerate(row):
            if char.isalnum():
                if colors.has_key(char):
                    color = colors[char]
                    if color_count[color]:
                        print '{}:{}:{} too many {} already'.format(
                            filename, i+1, j, char)
                        return None
                    color_count[color] = 1
                else:
                    color = len(colors)
                    colors[char] = color
                    color_count.append(0)

    for char, color in colors.iteritems():
        if not color_count[color]:
            print 'color {} has start but no end!'.format(char)
            return None
                    
    return colors

######################################################################

def make_color_clauses(puzzle, colors, color_var):

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
            neighbor_vars = [ color_var(ni, nj, solved_color) for
                              dir_bit, ni, nj in valid_neighbors(size, i, j) ]

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

    size = len(puzzle)
    dir_vars = dict()
    num_dir_vars = 0

    for i, j, char in explode(puzzle):

        if char.isalnum():
            continue

        neighbor_bits = (dir_bit for (dir_bit, ni, nj)
                         in valid_neighbors(size, i, j))

        cell_flags = reduce(operator.or_, neighbor_bits, 0)

        dir_vars[i,j] = dict()

        for code in DIR_CODES:
            if cell_flags & code == code:
                num_dir_vars += 1
                dir_vars[i,j][code] = start_var + num_dir_vars
 
    return dir_vars, num_dir_vars

######################################################################

def make_dir_clauses(puzzle, colors, color_var, dir_vars):

    dir_clauses = []
    num_colors = len(colors)
    size = len(puzzle)

    for i, j, char in explode(puzzle):
        
        if char.isalnum():
            continue
            
        cell_dir_dict = dir_vars[(i,j)]
        cell_dir_vars = cell_dir_dict.values()

        dir_clauses.append(cell_dir_vars)

        for da, db in allpairs(cell_dir_vars):
            dir_clauses.append([-da, -db])

        for dir_code, dir_var in cell_dir_dict.iteritems():
            
            for dir_bit, ni, nj in all_neighbors(i, j):

                for color in range(num_colors):
                    c1 = color_var(i, j, color)
                    c2 = color_var(ni, nj, color)
                    if dir_code & dir_bit:
                        dir_clauses.append([-dir_var, -c1, c2])
                        dir_clauses.append([-dir_var, c1, -c2])
                    elif valid_pos(size, ni, nj):
                        dir_clauses.append([-dir_var, -c1, -c2])

    return dir_clauses

######################################################################

def show_solution(puzzle, colors, color_var, dir_vars, sol):

    sol = set(sol)
    size = len(puzzle)

    row = []    

    for i, j, char in explode(puzzle):

        output = None
        
        for color_char, color in colors.iteritems():
            if color_var(i, j, color) in sol:
                assert output is None
                output = color_char

        assert output is not None

        boxchar = None

        if char.isalnum():
            boxchar = 'O'
        else:
            for dir_code, dir_var in dir_vars[i,j].iteritems():
                if dir_var in sol:
                    assert boxchar is None
                    boxchar = DIR_LOOKUP[dir_code]
            assert boxchar is not None

        if ANSI_LOOKUP.has_key(output):
            output = '\033[30;{}m{}\033[0m'.format(
                ANSI_LOOKUP[output], boxchar)

        sys.stdout.write(output)
        
        if j+1 == size:
            sys.stdout.write('\n')

######################################################################

def pyflow_solver():

    for filename in sys.argv[1:]:
        
        with open(filename, 'r') as infile:
            puzzle =  infile.read().splitlines()

        size = len(puzzle[0])
        puzzle = puzzle[:size]

        colors = make_colors(filename, puzzle)
        if colors is None:
            continue

        size = len(puzzle)
        num_colors = len(colors)

        print 'read {}x{} puzzle with {} colors from {}'.format(
            size, size, num_colors, filename)
        print

        num_cells = size**2
        num_color_vars = num_colors * num_cells

        def color_var(i, j, color):
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
        
        print 'generated {:,} clauses over {:,} color variables'.format(
            len(color_clauses), num_color_vars, grouping=True)
        
        print 'generated {:,} dir clauses over {:,} dir variables'.format(
            len(dir_clauses), num_dir_vars)

        print 'total {:,} clauses over {:,} variables'.format(
            len(clauses), num_vars)

        assemble_elapsed = (datetime.now() - start).total_seconds()
        total_elapsed = assemble_elapsed

        print 'assembled CNF in {:.3f} seconds'.format(assemble_elapsed)
        print

        start = datetime.now()
        num_solutions = 0

        for sol in pycosat.itersolve(clauses):

            end = datetime.now()
            solve_elapsed = (end - start).total_seconds()
            total_elapsed += solve_elapsed
            start = end
            num_solutions += 1

            print 'solver completed in {:.3f} seconds'.format(solve_elapsed)

            if type(sol) == list:
                print
                show_solution(puzzle, colors, color_var, dir_vars, sol)
                print

            if num_solutions > 1:
                print 'oh, no - more than one solution for', filename
                sys.exit(1)
                
        if not num_solutions:
            print 'no solutions found!'
            print

        print 'all done after {:.3f} seconds total'.format(
            total_elapsed)

                                                       
######################################################################
        
if __name__ == '__main__':
    pyflow_solver()

#include <stdio.h>
#include <stdint.h>
#include <locale.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <math.h>
#include <time.h>

#ifndef _WIN32
#include <unistd.h>
#include <sys/time.h>
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// ideas:
//
//   - TODO: figure out why things only work for bigger puzzles if
//           moves which are forced have zero cost.
//
//   - TODO: detect larger bottlenecks/choke points?
//
//   - TODO: try bidirectional search? (code is sort of in place but
//           seems to worsen things a lot.
//
//////////////////////////////////////////////////////////////////////

// Positions are 8-bit integers with 4 bits each for y, x.
enum {

  // Number to represent "not found"
  INVALID_POS = 0xff,
  
  // Maximum # of colors in a puzzle
  MAX_COLORS = 16,
  
  // Maximum valid size of a puzzle
  MAX_SIZE = 15,
  
  // Maximum # cells in a valid puzzle -- since we just use bit
  // shifting to do x/y, need to allocate space for 1 unused column.
  MAX_CELLS = (MAX_SIZE+1)*MAX_SIZE-1,

  // One million(ish) bytes
  MEGABYTE = 1024*1024,
  
};

// Various cell types, all but freespace have a color
enum {
  TYPE_FREE = 0, // Free space
  TYPE_PATH = 1, // Path between init & goal
  TYPE_INIT = 2, // Starting point
  TYPE_GOAL = 3  // Goal position
};

// Enumerate cardinal directions so we can loop over them
// RIGHT is increasing x, DOWN is increasing y.
enum {
  DIR_LEFT  = 0,
  DIR_RIGHT = 1,
  DIR_UP    = 2,
  DIR_DOWN  = 3
};

// Search termination results
enum {
  SEARCH_SUCCESS = 0,
  SEARCH_UNREACHABLE = 1,
  SEARCH_FULL = 2,
  SEARCH_IN_PROGRESS = 3,
};

// Represent the contents of a cell on the game board
typedef uint8_t cell_t;

// Represent a position within the game board
typedef uint8_t pos_t;

// Match color characters to ANSI color codes
typedef struct color_lookup_struct {
  char input_char;   // Color character
  char display_char; // Punctuation a la nethack
  int  ansi_code;    // ANSI color code
} color_lookup;

// Options for this program
typedef struct options_struct {

  int    display_quiet;
  int    display_diagnostics;
  int    display_animate;
  int    display_color;
  int    display_fast;
  
  int    node_check_touch;
  int    node_check_stranded;
  int    node_check_deadends;
  int    node_bottleneck_limit;
  int    node_penalize_exploration;
  
  int    order_autosort_colors;
  int    order_most_constrained;
  int    order_forced_first;
  int    order_random;
  
  int    search_astar_like;
  int    search_outside_in;
  size_t search_max_nodes;
  double search_max_mb;
  int    search_max_endpoint;
  
} options_t;

// Static information about a puzzle layout -- anything that does not
// change as the puzzle is solved is stored here.
typedef struct game_info_struct {

  // Index in color_dict table of codes
  int    color_ids[MAX_COLORS];

  // Color order
  int    color_order[MAX_COLORS];

  // Length/width of game board
  size_t size;

  // Number of colors present
  size_t num_colors;

  // Color table for looking up color ID
  uint8_t color_tbl[128];

  // Was user order specified?
  int user_order;
  
} game_info_t;

// Incremental game state structure for solving -- this is what gets
// written as the search progresses, one state per search node
typedef struct game_state_struct {

  // State of each cell in the world; a little wasteful to duplicate,
  // since only one changes on each move, but necessary for BFS or A*
  // (would not be needed for depth-first search).
  cell_t   cells[MAX_CELLS];

  // Head position
  pos_t    pos[MAX_COLORS][2];

  // How many free cells?
  uint8_t  num_free;

  // Which was the last color / endpoint
  cell_t  last_move;

  // Bitflag indicating whether each color has been completed or not
  // (cur_pos is adjacent to goal_pos).
  uint16_t completed;
  
} game_state_t;

typedef struct color_features_struct {
  int index;
  int user_index;
  int wall_dist[2];
  int min_dist;
} color_features_t;

// Disjoint-set data structure for connected component analysis of free
// space (see region_ functions).
typedef struct region_struct {
  // Parent index (or INVALID_POS) if no non-free space
  pos_t parent;
  // Rank (see wikipedia article).
  uint8_t rank;
} region_t;

// Search node for A* / BFS.
typedef struct tree_node_struct {
  game_state_t state;              // Current game state
  double cost_to_come;             // Cost to come (ignored for BFS)
  double cost_to_go;               // Heuristic cost (ignored for BFS)
  struct tree_node_struct* parent; // Parent of this node (may be NULL)
} tree_node_t;

// Strategy is to pre-allocate a big block of nodes in advance, and
// hand them out in order.
typedef struct node_storage_struct {
  tree_node_t* start; // Allocated block
  size_t capacity;    // How many nodes did we allocate?
  size_t count;       // How many did we give out?
} node_storage_t;

// Data structure for heap based priority queue
typedef struct heapq_struct {
  tree_node_t** start; // Array of node pointers
  size_t capacity;     // Maximum allowable queue size
  size_t count;        // Number enqueued
} heapq_t;

// First in, first-out queue implemented as an array of pointers.
typedef struct fifo_struct {
  tree_node_t** start; // Array of node pointers
  size_t capacity;     // Maximum number of things to enqueue ever
  size_t count;        // Total enqueued (next one will go into start[count])
  size_t next;         // Next index to dequeue
} fifo_t;

// Union struct for passing around queues.
typedef union queue_union {
  heapq_t heapq;
  fifo_t  fifo;
} queue_t;

// Function pointers for either type of queue
queue_t (*queue_create)(size_t) = 0;
void (*queue_enqueue)(queue_t*, tree_node_t*) = 0;
tree_node_t* (*queue_deque)(queue_t*) = 0;
void (*queue_destroy)(queue_t*) = 0;
int (*queue_empty)(const queue_t*) = 0;
const tree_node_t* (*queue_peek)(const queue_t*) = 0;

//////////////////////////////////////////////////////////////////////

// For succinct printing of search results
const char SEARCH_RESULT_CHARS[4] = "suf?";

// For verbose printing of search results
const char* SEARCH_RESULT_STRINGS[4] = {
  "successful",
  "unsolvable",
  "out of memory",
  "in progress"
};

// Was gonna try some unicode magic but meh
const char* BLOCK_CHAR = "#";

// For visualizing cardinal directions ordered by the enum above.
const char DIR_CHARS[4] = "<>^v";

// x, y, pos coordinates for each direction
const int DIR_DELTA[4][3] = {
  { -1, 0, -1 },
  {  1, 0,  1 },
  {  0, -1, -16 },
  {  0, 1, 16 }
};

// Look-up table mapping characters in puzzle definitions to ANSI
// colors.
const color_lookup color_dict[MAX_COLORS] = {
  { 'R', 'o', 101 }, // red
  { 'B', '+', 104 }, // blue
  { 'Y', '@', 103 }, // yellow
  { 'G', '*',  42 }, // green
  { 'O', 'x',  43 }, // orange
  { 'C', '%', 106 }, // cyan
  { 'M', '?', 105 }, // magenta
  { 'm', 'v',  41 }, // maroon
  { 'P', '^',  45 }, // purple
  { 'A', '=', 100 }, // gray
  { 'W', '~', 107 }, // white
  { 'g', '-', 102 }, // bright green
  { 'T', '$',  47 }, // tan
  { 'b', '"',  44 }, // dark blue
  { 'c', '&',  46 }, // dark cyan
  { 'p', '.',  35 }, // pink?
};

// Global options struct gets setup during main
options_t g_options;

//////////////////////////////////////////////////////////////////////
// Return the current time as a double. Don't actually care what zero
// is cause we will just offset.

double now() {
  
#ifdef _WIN32
  union {
    LONG_LONG ns100; /*time since 1 Jan 1601 in 100ns units */
    FILETIME ft;
  } now;
  GetSystemTimeAsFileTime (&now.ft);
  return (double)now.ns100 * 1e-7; // 100 nanoseconds = 0.1 microsecond
#else
  struct timeval tv;
  gettimeofday(&tv, 0);
  return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
#endif

}

//////////////////////////////////////////////////////////////////////
// Peform lookup in color_dict above

int get_color_id(char c) {
  for (int i=0; i<MAX_COLORS; ++i) {
    if (color_dict[i].input_char == c) {
      return i;
    }
  }
  return -1;
}

//////////////////////////////////////////////////////////////////////
// Detect whether terminal supports color & cursor commands

int terminal_has_color() {

#ifdef _WIN32

  return 0;

#else
  
  if (!isatty(STDOUT_FILENO)) {
    return 0;
  } 

  char* term = getenv("TERM");
  if (!term) { return 0; }
  
  return strstr(term, "xterm") == term || strstr(term, "rxvt") == term;

#endif
  
}

//////////////////////////////////////////////////////////////////////
// Emit color string for index into color_dict table above

const char* set_color_str(int id) {
  if (g_options.display_color) {
    static char buf[256];
    snprintf(buf, 256, "\033[30;%dm", color_dict[id].ansi_code);
    return buf; 
  } else {
    return "";
  }
}

//////////////////////////////////////////////////////////////////////
// Reset terminal color to default

const char* reset_color_str() {
  if (g_options.display_color) {
    return "\033[0m";
  } else {
    return "";
  }
}

//////////////////////////////////////////////////////////////////////
// Clear screen and set cursor pos to 0,0

const char* unprint_board(const game_info_t* info) {
  if (g_options.display_color) {
    static char buf[256];
    snprintf(buf, 256, "\033[%zuA\033[%zuD",
             info->size+2, info->size+2);
    return buf;
  } else {
    return "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
  }
}

//////////////////////////////////////////////////////////////////////
// Create a delay

void delay_seconds(double s) {
  if (g_options.display_fast) { s /= 4.0; }
#ifdef _WIN32
  // TODO: find win32 equivalent of usleep?
#else
  usleep((size_t)(s * 1e6));
#endif
}

//////////////////////////////////////////////////////////////////////
// Create a 8-bit position from 2 4-bit x,y coordinates

pos_t pos_from_coords(pos_t x, pos_t y) {
  return ((y & 0xf) << 4) | (x & 0xf);
}

//////////////////////////////////////////////////////////////////////
// Split 8-bit position into 4-bit x & y coords

void pos_get_coords(pos_t p, int* x, int* y) {
  *x = p & 0xf;
  *y = (p >> 4) & 0xf;
}

//////////////////////////////////////////////////////////////////////
// Are the two positions a single cardinal direction move apart?

int pos_is_adjacent(pos_t a, pos_t b) {
  int diff = a - b;
  diff = diff < 0 ? -diff : diff;
  return diff == 1 || diff == 16;
}

//////////////////////////////////////////////////////////////////////
// Are the coords on the map?

int coords_valid(const game_info_t* info,
                 int x, int y) {

  return (x >= 0 && x < (int)info->size &&
          y >= 0 && y < (int)info->size);

}

//////////////////////////////////////////////////////////////////////
// Compote an offset as a position and return whether valid or not

pos_t offset_pos(const game_info_t* info,
                 int x, int y, int dir) {

    int offset_x = x + DIR_DELTA[dir][0];
    int offset_y = y + DIR_DELTA[dir][1];

    return coords_valid(info, offset_x, offset_y) ?
      pos_from_coords(offset_x, offset_y) : INVALID_POS;
  
}

//////////////////////////////////////////////////////////////////////
// Compote an offset as a position and return whether valid or not

pos_t pos_offset_pos(const game_info_t* info,
                     pos_t pos, int dir) {

  int x, y;
  pos_get_coords(pos, &x, &y);
  return offset_pos(info, x, y, dir);

}

//////////////////////////////////////////////////////////////////////
// Get the distance from the wall for x, y coords

int get_wall_dist(const game_info_t* info,
                  int x, int y) {

  int p[2] = { x, y };
  int d[2];

  for (int i=0; i<2; ++i) {
    int d0 = p[i];
    int d1 = info->size - 1 - p[i];
    d[i] = d0 < d1 ? d0 : d1;
  }

  return d[0] < d[1] ? d[0] : d[1];

}

//////////////////////////////////////////////////////////////////////
// Get the distance from the wall for 8-bit position

int pos_get_wall_dist(const game_info_t* info,
                      pos_t pos) {

  int x, y;

  pos_get_coords(pos, &x, &y);
  return get_wall_dist(info, x, y);

}

//////////////////////////////////////////////////////////////////////
// Create a cell from a 2-bit type, a 4-bit color, and a 2-bit
// direction.

cell_t cell_create(uint8_t type, uint8_t color, uint8_t dir) {
  return ((color & 0xf) << 4) | ((dir & 0x3) << 2) | (type & 0x3);
}

//////////////////////////////////////////////////////////////////////
// Get the type from a cell value

uint8_t cell_get_type(cell_t c) {
  return c & 0x3;
}

//////////////////////////////////////////////////////////////////////
// Get the direction from a cell value

uint8_t cell_get_direction(cell_t c) {
  return (c >> 2) & 0x3;
}

//////////////////////////////////////////////////////////////////////
// Get the color from a cell value

uint8_t cell_get_color(cell_t c) {
  return (c >> 4) & 0xf;
}

//////////////////////////////////////////////////////////////////////
// For displaying a color nicely

const char* color_name_str(const game_info_t* info,
                            int color) {

  int id = info->color_ids[color];
  char i = color_dict[id].input_char;
  char d = color_dict[id].display_char;

  static char buf[1024];

  snprintf(buf, 1024, "%s%c%s",
           set_color_str(id),
           g_options.display_color ? i : d,
           reset_color_str());

  return buf;
  

}

//////////////////////////////////////////////////////////////////////
// For displaying a cell nicely

const char* color_cell_str(const game_info_t* info,
                           cell_t cell) {

  int type = cell_get_type(cell);
  int color = cell_get_color(cell);
  int dir = cell_get_direction(cell);
  
  int id = info->color_ids[color];
  
  char c = color_dict[id].display_char;

  static char buf[1024];
  
  switch (type) {
  case TYPE_FREE:
    snprintf(buf, 1024, " ");
    break;
  case TYPE_PATH:
    snprintf(buf, 1024, "%s%c%s",
             set_color_str(id),
             g_options.display_color ? DIR_CHARS[dir] : c,
             reset_color_str());
    break;
  default:
    snprintf(buf, 1024, "%s%c%s",
             set_color_str(info->color_ids[color]),
             g_options.display_color ? (type == TYPE_INIT ? 'o' : 'O') : c,
             reset_color_str());
  }

  return buf;

}
  
//////////////////////////////////////////////////////////////////////
// Consider whether the given move is valid.

int game_can_move(const game_info_t* info,
                  const game_state_t* state,
                  int color, int dir, int endpoint) {

  // Make sure color is valid
  assert(color < info->num_colors);

  assert(!(state->completed & (1 << color)));

  // Get cur pos x, y
  int cur_x, cur_y;
  pos_get_coords(state->pos[color][endpoint], &cur_x, &cur_y);

  int sign = (endpoint == 0 ? 1 : -1);

  // Get new x, y
  int new_x = cur_x + sign * DIR_DELTA[dir][0];
  int new_y = cur_y + sign * DIR_DELTA[dir][1];

  // If outside bounds, not legal
  if (new_x < 0 || new_x >= info->size ||
      new_y < 0 || new_y >= info->size) {
    return 0;
  }

  // Create a new position
  pos_t new_pos = pos_from_coords(new_x, new_y);
  assert( new_pos < MAX_CELLS );

  // Must be empty (TYPE_FREE)
  if (state->cells[new_pos]) {
    return 0;
  }

  if (g_options.node_check_touch) {
    
    // All puzzles are designed so that a new path segment is adjacent
    // to at most one path segment of the same color -- the predecessor
    // to the new segment. We check this by iterating over the
    // neighbors.
    for (int dir=0; dir<4; ++dir) {

      // Assemble position
      pos_t neighbor_pos = offset_pos(info, new_x, new_y, dir);

      // If valid non-empty cell and not cur_pos and not goal_pos and
      // has our color, then fail
      if (neighbor_pos != INVALID_POS && 
          state->cells[neighbor_pos] &&
          neighbor_pos != state->pos[color][0] && 
          neighbor_pos != state->pos[color][1] &&
          cell_get_color(state->cells[neighbor_pos]) == color) {
        return 0;
      }
    
    }

  }
  
  // It's valid
  return 1;

}


//////////////////////////////////////////////////////////////////////
// Print out game board

void game_print(const game_info_t* info,
                const game_state_t* state) {

  printf("%s", BLOCK_CHAR);
  for (size_t x=0; x<info->size; ++x) {
    printf("%s", BLOCK_CHAR);
  }
  printf("%s\n", BLOCK_CHAR);

  for (size_t y=0; y<info->size; ++y) {
    printf("%s", BLOCK_CHAR);
    for (size_t x=0; x<info->size; ++x) {
      cell_t cell = state->cells[pos_from_coords(x, y)];
      printf("%s", color_cell_str(info, cell));
    }
    printf("%s\n", BLOCK_CHAR);
  }

  printf("%s", BLOCK_CHAR);
  for (size_t x=0; x<info->size; ++x) {
    printf("%s", BLOCK_CHAR);
  }
  printf("%s\n", BLOCK_CHAR);
  
}

//////////////////////////////////////////////////////////////////////
// Return the number of free spaces around an x, y position

int game_num_free_coords(const game_info_t* info,
                         const game_state_t* state,
                         int x, int y) {

  int num_free = 0;
  
  for (int dir=0; dir<4; ++dir) {
    pos_t neighbor_pos = offset_pos(info, x, y, dir);
    if (neighbor_pos != INVALID_POS &&
        state->cells[neighbor_pos] == 0) {
      ++num_free;
    }
  }

  return num_free;

}

//////////////////////////////////////////////////////////////////////
// Return the number of free spaces around an 8-bit position

int game_num_free_pos(const game_info_t* info,
                      const game_state_t* state,
                      pos_t pos) {

  int x, y;

  pos_get_coords(pos, &x, &y);
  return game_num_free_coords(info, state, x, y);

}


//////////////////////////////////////////////////////////////////////
// Update the game state to make the given move.

double game_make_move(const game_info_t* info,
                      game_state_t* state, 
                      int color, int dir, int endpoint) {


  // Make sure valid color
  assert(color < info->num_colors);
  assert(endpoint == 0);

  // Update the cell with the new cell value
  cell_t move = cell_create(TYPE_PATH, color, dir);
  
  // Get current x, y
  int cur_x, cur_y;
  pos_get_coords(state->pos[color][endpoint], &cur_x, &cur_y);

  int sign = (endpoint == 0 ? 1 : -1);

  // Assemble new x, y
  int new_x = cur_x + sign * DIR_DELTA[dir][0];
  int new_y = cur_y + sign * DIR_DELTA[dir][1];

  // Make sure valid
  assert( new_x >= 0 && new_x < (int)info->size &&
          new_y >= 0 && new_y < (int)info->size );

  // Make position
  pos_t new_pos = pos_from_coords(new_x, new_y);
  assert( new_pos < MAX_CELLS );

  // Make sure it's empty
  assert( state->cells[new_pos] == 0 );


  // Update cells and new pos
  state->cells[new_pos] = move;
  state->pos[color][endpoint] = new_pos;
  --state->num_free;
  state->last_move = cell_create(endpoint == 0 ? TYPE_INIT : TYPE_GOAL, color, dir);

  double action_cost = 1;

  if (pos_is_adjacent(state->pos[color][0],
                      state->pos[color][1])) {
    
    state->completed |= (1 << color);
    action_cost = 0;
    
  } else {
  
    int num_free = game_num_free_coords(info, state,
                                        new_x, new_y);

    if (num_free == 1) {
      action_cost = 0;
    } else if (g_options.node_penalize_exploration && num_free == 2) {
      action_cost = 2;
    }

  }
  
  return action_cost;

}

//////////////////////////////////////////////////////////////////////
// Read game board from text file

int game_read(const char* filename,
              game_info_t* info,
              game_state_t* state) {

  FILE* fp = fopen(filename, "r");

  if (!fp) {
    fprintf(stderr, "error opening %s\n", filename);
    return 0;
  }

  memset(info, 0, sizeof(game_info_t));
  memset(state, 0, sizeof(game_state_t));
  
  memset(state->pos, 0xff, sizeof(state->pos));

  state->last_move = 0;

  size_t y=0;

  char buf[MAX_SIZE+2];


  memset(info->color_tbl, 0xff, sizeof(info->color_tbl));

  while (info->size == 0 || y < info->size) {

    char* s = fgets(buf, MAX_SIZE+1, fp);
    size_t l = s ? strlen(s) : 0;
    
    if (!s) {
      fprintf(stderr, "%s:%zu: unexpected EOF\n", filename, y+1);
      fclose(fp);
      return 0;
    } else if (s[l-1] != '\n') {
      fprintf(stderr, "%s:%zu line too long\n", filename, y+1);
      fclose(fp);
      return 0;
    }

    if (info->size == 0) {
      if (l < 6) {
        fprintf(stderr, "%s:1: expected at least 5 characters before newline\n",
                filename);
        fclose(fp);
        return 0;
      } else if (l-1 > MAX_SIZE) {
        fprintf(stderr, "%s:1: size too big!\n", filename);
        fclose(fp);
        return 0;
      }
      info->size = l-1;
    } else if (l != info->size + 1) {
      fprintf(stderr, "%s:%zu: wrong number of characters before newline "
              "(expected %zu, but got %zu)\n",
              filename, y+1,
              info->size, l-1);
      fclose(fp);
      return 0;
    }

    for (size_t x=0; x<info->size; ++x) {
      
      uint8_t c = s[x];
      
      if (isalpha(c)) {

        pos_t pos = pos_from_coords(x, y);
        assert(pos < MAX_CELLS);

        int color = info->color_tbl[c];
        
        if (color >= info->num_colors) {

          color = info->num_colors;
          
          if (info->num_colors == MAX_COLORS) {
            fprintf(stderr, "%s:%zu: can't use color %c"
                    "- too many colors!\n",
                    filename, y+1, c);
            fclose(fp);
            return 0;

          }
          
          int id = get_color_id(c);
          if (id < 0) {
            fprintf(stderr, "%s:%zu: unrecognized color %c\n",
                    filename, y+1, c);
            fclose(fp);
            return 0;
          }

          info->color_ids[color] = id;
          info->color_order[color] = color;
          
          ++info->num_colors;
          info->color_tbl[c] = color;
          state->pos[color][0] = pos;
          state->cells[pos] = cell_create(TYPE_INIT, color, 0);

        } else {

          if (state->pos[color][1] != INVALID_POS) {
            fprintf(stderr, "%s:%zu too many %c already!\n",
                    filename, y+1, c);
            fclose(fp);
            return 0;
          }
          state->pos[color][1] = pos;
          state->cells[pos] = cell_create(TYPE_GOAL, color, 0);

        }
      } else {

        ++state->num_free;

      }
    }
    
    ++y;
  }

  fclose(fp);

  if (!info->num_colors) {
    fprintf(stderr, "empty map!\n");
    return 0;
  }

  for (size_t color=0; color<info->num_colors; ++color) {

    if (state->pos[color][1] == INVALID_POS) {
      game_print(info, state);
      fprintf(stderr, "\n\n%s: color %s has start but no end\n",
              filename,
              color_name_str(info, color));
      return 0;
    }

    if (g_options.search_outside_in) {

      int wd[2];

      for (int i=0; i<2; ++i) {
        wd[i] = pos_get_wall_dist(info, state->pos[color][i]);
      }

      if (wd[1] < wd[0]) {
        pos_t tmp_pos = state->pos[color][0];
        state->pos[color][0] = state->pos[color][1];
        state->pos[color][1] = tmp_pos;
        state->cells[state->pos[color][0]] = cell_create(TYPE_INIT, color, 0);
        state->cells[state->pos[color][1]] = cell_create(TYPE_GOAL, color, 0);
      }

    }

  }
  
  return 1;

}

//////////////////////////////////////////////////////////////////////
// Read hint file

int game_read_hint(const game_info_t* info,
                   const game_state_t* state,
                   const char* filename,
                   uint8_t hint[MAX_CELLS]) {
  
  memset(hint, 0xff, MAX_CELLS);

  FILE* fp = fopen(filename, "r");

  if (!fp) {
    fprintf(stderr, "error opening %s\n", filename);
    return 0;
  }

  char buf[MAX_SIZE+2];

  for (size_t y=0; y<info->size; ++y) {
    char* s = fgets(buf, info->size+2, fp);
    if (!s) { break; }
    size_t l = strlen(s);
    if (l > info->size+1 || s[l-1] != '\n') {
      fprintf(stderr, "%s:%zu: line too long!\n", filename, y+1);
      fclose(fp);
      return 0;
    }
    for (size_t x=0; x<l-1; ++x) {
      uint8_t c = buf[x];
      if (isalpha(c)) {
        int color = c > 127 ? 0xff : info->color_tbl[c];
        if (color >= info->num_colors) {
          fprintf(stderr, "%s:%zu: color %c not found!\n", filename, y+1, c);
          fclose(fp);
          return 0;
        }
        int pos = pos_from_coords(x, y);
        hint[pos] = color;
      }
    }
  }

  fclose(fp);
  return 1;
  
}

//////////////////////////////////////////////////////////////////////
// Pick the next color to move deterministically

int game_next_move_color(const game_info_t* info,
                         const game_state_t* state,
                         int* endpoint_out) {

  *endpoint_out = 0;

  size_t last_color = state->last_move ? cell_get_color(state->last_move) : MAX_COLORS;

  if (last_color < info->num_colors &&
      !(state->completed & (1 << last_color))) {
    return last_color;
  }

  if (!info->user_order &&
      g_options.order_most_constrained) {

    size_t best_color = -1;
    int best_endpoint = 0;
    int best_free = 4;
    
    for (size_t i=0; i<info->num_colors; ++i) {

      for (int endpoint=0; endpoint<g_options.search_max_endpoint; ++endpoint) {

        int color = info->color_order[i];
        if (state->completed & (1 << color)) { continue; }

        int num_free = game_num_free_pos(info, state,
                                         state->pos[color][endpoint]);
                
        if (num_free < best_free) {
          best_free = num_free;
          best_color = color;
          best_endpoint = endpoint;
        }

      }

    }

    assert(best_color < info->num_colors);

    *endpoint_out = best_endpoint;
    return best_color;
    
  } else {

    for (size_t i=0; i<info->num_colors; ++i) {
      int color = info->color_order[i];
      if (state->completed & (1 << color)) { continue; }
      return color;
    }

    assert(0 && "unreachable code");
    return -1;
    
  } 

}

//////////////////////////////////////////////////////////////////////
// Compare 2 ints

int cmp(int a, int b) {
  return a < b ? -1 : a > b ? 1 : 0;
}

//////////////////////////////////////////////////////////////////////
// For sorting colors

int color_features_compare(const void* vptr_a, const void* vptr_b) {

  const color_features_t* a = (const color_features_t*)vptr_a;
  const color_features_t* b = (const color_features_t*)vptr_b;

  int u = cmp(a->user_index, b->user_index);
  if (u) { return u; }

  int w = cmp(a->wall_dist[0], b->wall_dist[0]);
  if (w) { return w; }

  int g = -cmp(a->wall_dist[1], b->wall_dist[1]);
  if (g) { return g; }

  return -cmp(a->min_dist, b->min_dist);

}

//////////////////////////////////////////////////////////////////////
// Place the game colors into a set order

void game_order_colors(game_info_t* info,
                       game_state_t* state,
                       const char* user_order) {

  if (g_options.order_random) {
    
    srand(now() * 1e6);
    
    for (size_t i=info->num_colors-1; i>0; --i) {
      size_t j = rand() % (i+1);
      int tmp = info->color_order[i];
      info->color_order[i] = info->color_order[j];
      info->color_order[j] = tmp;
    }

  } else { // not random

    color_features_t cf[MAX_COLORS];
    memset(cf, 0, sizeof(cf));

    for (size_t color=0; color<info->num_colors; ++color) {
      cf[color].user_index = MAX_COLORS;
    }
    
    if (g_options.order_autosort_colors) {

      for (size_t color=0; color<info->num_colors; ++color) {

        int x[2], y[2];

        for (int i=0; i<2; ++i) {
          pos_get_coords(state->pos[color][i], x+i, y+i);
          cf[color].wall_dist[i] = get_wall_dist(info, x[i], y[i]);
        }

        int dx = abs(x[1]-x[0]);
        int dy = abs(y[1]-y[0]);
      
        cf[color].index = color;
        cf[color].min_dist = dx + dy;

      }

    }

    if (user_order) {
      
      for (size_t k=0; user_order[k]; ++k) {
        uint8_t c = user_order[k];
        int color = c < 127 ? info->color_tbl[c] : MAX_COLORS;
        if (color >= info->num_colors) {
          fprintf(stderr, "error ordering colors: %c not in puzzle\n", c);
          exit(1);
        }
        if (cf[color].user_index < info->num_colors) {
          fprintf(stderr, "error ordering colors: %c already used\n", c);
          exit(1);
        }
        cf[color].user_index = k;
      }

      info->user_order = 1;

    }

    mergesort(cf, info->num_colors, sizeof(color_features_t),
              color_features_compare);

    for (size_t i=0; i<info->num_colors; ++i) {
      info->color_order[i] = cf[i].index;
    }
    
  }

  if (!g_options.display_quiet) {
    
    if (g_options.order_most_constrained && !user_order) {
      printf("will choose color by most constrained\n");
    } else {
      printf("will choose colors in order: ");
      for (size_t i=0; i<info->num_colors; ++i) {
        int color = info->color_order[i];
        printf("%s", color_name_str(info, color));
      }
      printf("\n");
    }

  }

}

//////////////////////////////////////////////////////////////////////
// This and other connected component analysis functions from
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
//
// Create a new set with a single member:

region_t region_create(pos_t pos) {
  region_t rval;
  rval.parent = pos;
  rval.rank = 0;
  return rval;
}

//////////////////////////////////////////////////////////////////////
// Loop up root region for the given index p

pos_t region_find(region_t* regions, pos_t p) {
  if (regions[p].parent != INVALID_POS &&
      regions[p].parent != p) {
    assert(p != INVALID_POS);
    assert(p < MAX_CELLS);
    regions[p].parent = region_find(regions, regions[p].parent);
  }
  return regions[p].parent;
}

//////////////////////////////////////////////////////////////////////
// Merge two regions

void region_unite(region_t* regions,
                  pos_t a, pos_t b) {

  pos_t root_a = region_find(regions, a);
  pos_t root_b = region_find(regions, b);

  if (root_a == root_b) { return; }

  if (regions[root_a].rank < regions[root_b].rank) {
    regions[root_a].parent = root_b;
  } else if (regions[root_a].rank > regions[root_b].rank) {
    regions[root_b].parent = root_a;
  } else {
    regions[root_b].parent = root_a;
    ++regions[root_a].rank;
  }

}

//////////////////////////////////////////////////////////////////////
// Perform connected components analysis on game board. This is a
// 2-pass operation: one to build and merge the disjoint-set data
// structures, and another to re-index them so each unique region of
// free space gets its own index, starting at zero. Returns the number
// of freespace regions.

size_t game_build_regions(const game_info_t* info,
                          const game_state_t* state,
                          uint8_t rmap[MAX_CELLS]) {

  region_t regions[MAX_CELLS];
  
  // 1 pass to build regions
  for (size_t y=0; y<info->size; ++y) {
    for (size_t x=0; x<info->size; ++x) {
      pos_t pos = pos_from_coords(x, y);
      if (state->cells[pos]) {
        regions[pos] = region_create(INVALID_POS);
      } else {
        regions[pos] = region_create(pos);
        if (x) {
          pos_t pl = pos_from_coords(x-1, y);
          if (!state->cells[pl]) {
            region_unite(regions, pos, pl);
          }
        }
        if (y) {
          pos_t pu = pos_from_coords(x, y-1);
          if (!state->cells[pu]) {
            region_unite(regions, pos, pu);
          }
        }
      }
    }
  }

  uint8_t rlookup[MAX_CELLS];
  size_t rcount = 0;
  
  memset(rlookup, 0xff, sizeof(rlookup));
  memset(rmap, 0xff, MAX_CELLS);

  // 2nd pass to order regions
  for (size_t y=0; y<info->size; ++y) {
    for (size_t x=0; x<info->size; ++x) {
      pos_t pos = pos_from_coords(x, y);
      pos_t root = region_find(regions, pos);
      if (root != INVALID_POS) {
        if (rlookup[root] == INVALID_POS) {
          rlookup[root] = rcount++;
        }
        rmap[pos] = rlookup[root];
      } else {
        rmap[pos] = INVALID_POS;
      }
    }
  }

  return rcount;

}

//////////////////////////////////////////////////////////////////////
// Helper function for game_regions_ok below -- this is used to add
// the current color bit flag to the regions adjacent to the current
// or goal position. 

void game_regions_add_color(const game_info_t* info,
                            const game_state_t* state,
                            const uint8_t rmap[MAX_CELLS],
                            pos_t pos,
                            uint16_t cflag,
                            uint16_t* rflags) {

  for (int dir=0; dir<4; ++dir) {

    pos_t neighbor_pos = pos_offset_pos(info, pos, dir);

    if (neighbor_pos != INVALID_POS) {

      // find out what region it is in
      int neighbor_region = rmap[neighbor_pos];

      // if it is in a valid region
      if (neighbor_region != INVALID_POS) {
        // add this color to the region
        rflags[neighbor_region] |= cflag;
      }

    }
    
  }
  
}

//////////////////////////////////////////////////////////////////////
// Check for dead-end regions of freespace where there is no way to
// put an active path into and out of it. Any freespace node which
// has only one free neighbor represents such a dead end. For the
// purposes of this check, cur and goal positions count as "free".

int game_regions_deadends(const game_info_t* info,
                          const game_state_t* state,
                          size_t rcount,
                          const uint8_t rmap[MAX_CELLS]) {

  for (size_t y=0; y<info->size; ++y) {
    for (size_t x=0; x<info->size; ++x) {

      pos_t pos = pos_from_coords(x, y);
      if (rmap[pos] >= rcount) { continue; }

      int num_free = 0;

      for (int dir=0; dir<4; ++dir) {
        pos_t neighbor_pos = offset_pos(info, x, y, dir);
        if (neighbor_pos != INVALID_POS) {
          if (rmap[neighbor_pos] == rmap[pos]) {
            ++num_free;
          } else {
            for (size_t color=0; color<info->num_colors; ++color) {
              if (state->completed & (1 << color)) {
                continue;
              }
              if (neighbor_pos == state->pos[color][0] ||
                  neighbor_pos == state->pos[color][1]) {
                ++num_free;
              }
            }
                                                                 
          }
        }
      }

      if (num_free <= 1) {

        return 1;
        
      }
      
    }
  }

  return 0;

}
                        

//////////////////////////////////////////////////////////////////////
// Check the results of the connected-component analysis to make sure
// that every color can get solved and no freespace is isolated

int game_regions_stranded(const game_info_t* info,
                          const game_state_t* state,
                          size_t rcount,
                          const uint8_t rmap[MAX_CELLS],
                          size_t chokepoint_color,
                          int max_stranded) {

  // For each region, we have bitflags to track whether current or
  // goal position is adjacent to the region. These get initted to 0.
  uint16_t cur_rflags[rcount];
  uint16_t goal_rflags[rcount];

  memset(cur_rflags, 0, sizeof(cur_rflags));
  memset(goal_rflags, 0, sizeof(goal_rflags));

  int num_stranded = 0;
  uint16_t colors_stranded = 0;

  int for_chokepoint = chokepoint_color < info->num_colors;

  // For each color, figure out which regions touch its current and
  // goal position, and make sure no color is "stranded"
  for (int color=0; color<info->num_colors; ++color) {

    uint16_t cflag = (1 << color);

    // No worries if completed:
    if ((state->completed & cflag) || color == chokepoint_color) {
      continue;
    }

    // Add color flag to all regions for cur_pos
    game_regions_add_color(info, state, rmap,
                           state->pos[color][0],
                           cflag, cur_rflags);

    // Add color flag to all regions for goal_pos
    game_regions_add_color(info, state, rmap,
                           state->pos[color][1],
                           cflag, goal_rflags);

    // Ensure this color is not "stranded" -- at least region must
    // touch each non-completed color for both current and goal.
    size_t r;

    // for each region
    for (r=0; r<rcount; ++r) {
      // see if this region touches the color
      if ((cur_rflags[r] & cflag) &&
          (goal_rflags[r] & cflag)) {
        break;
      }
    }

    // There was no region that touched both current and goal,
    // unsolvable from here.
    if (r == rcount) {
      colors_stranded |= cflag;
      if (++num_stranded >= max_stranded) {
        return colors_stranded;
      }
    }

  }

  if (!for_chokepoint) {

    // For each region, make sure that there is at least one color whose
    // current and goal positions touch it; otherwise, the region is
    // stranded.
    for (size_t r=0; r<rcount; ++r) {
      if (!(cur_rflags[r] & goal_rflags[r])) {
        return -1;
      }
    }

  }
  
  // Everything a-ok.
  return 0;
  
}

//////////////////////////////////////////////////////////////////////
// Print connected components of freespace
                        
void game_print_regions(const game_info_t* info,
                        const game_state_t* state,
                        uint8_t rmap[MAX_CELLS]) {

  printf("%s", BLOCK_CHAR);
  for (size_t x=0; x<info->size; ++x) {
    printf("%s", BLOCK_CHAR);
  }
  printf("%s\n", BLOCK_CHAR);

  for (size_t y=0; y<info->size; ++y) {
    printf("%s", BLOCK_CHAR);
    for (size_t x=0; x<info->size; ++x) {
      pos_t pos = pos_from_coords(x, y);
      pos_t rid = rmap[pos];
      if (!state->cells[pos]) {
        assert(rid != INVALID_POS);
        char c = 65 + rid % 60;
        printf("%s%c%s",
               set_color_str(rid % MAX_COLORS), c, reset_color_str());
      } else {
        assert(rid == INVALID_POS);
        printf(" ");
      }
    }
    printf("%s\n", BLOCK_CHAR);
  }

  printf("%s", BLOCK_CHAR);
  for (size_t x=0; x<info->size; ++x) {
    printf("%s", BLOCK_CHAR);
  }
  printf("%s\n", BLOCK_CHAR);
  
  printf("\n");
  
}

//////////////////////////////////////////////////////////////////////
// Helper function for game_find_forced below.

int game_is_forced(const game_info_t* info,
                   const game_state_t* state,
                   int color, int endpoint,
                   pos_t pos) {

  int num_free = 0;
  int num_other_endpoints = 0;

  for (int dir=0; dir<4; ++dir) {
    pos_t neighbor_pos = pos_offset_pos(info, pos, dir);
    if (neighbor_pos == INVALID_POS ||
        neighbor_pos == state->pos[color][endpoint]) {
      continue;
    } else if (state->cells[neighbor_pos] == 0) {
      ++num_free;
    } else {
      for (size_t other_color=0; other_color<info->num_colors; ++other_color) {
        if (other_color == color) { continue; }
        if (state->completed & (1 << other_color)) { continue; }
        if (neighbor_pos == state->pos[other_color][0] ||
            neighbor_pos == state->pos[other_color][1]) {
          ++num_other_endpoints;
        }
      }
    } 
  } // for each neighbor

  return (num_free == 1 && num_other_endpoints == 0);

}
                                         

//////////////////////////////////////////////////////////////////////
// Find a forced move. This could be optimized to not search all
// colors all the time, maybe?

int game_find_forced(const game_info_t* info,
                     const game_state_t* state,
                     int* forced_color,
                     int* forced_dir,
                     int* forced_endpoint) {

  // if there is a freespace next to an endpoint and the freespace has
  // only one free neighbor, we must extend the endpoint into it.

  for (size_t i=0; i<info->num_colors; ++i) {

    size_t color = info->color_order[i];

    if (state->completed & (1 << color)) { continue; }

    for (int endpoint=0; endpoint<g_options.search_max_endpoint; ++endpoint) {

      for (int dir=0; dir<4; ++dir) {

        pos_t neighbor_pos = pos_offset_pos(info, state->pos[color][endpoint], dir);
        if (neighbor_pos == INVALID_POS) { continue; }

        if (state->cells[neighbor_pos] == 0) {

          if (game_is_forced(info, state, color, endpoint, neighbor_pos)) {

            *forced_color = color;
            *forced_dir = dir;
            *forced_endpoint = endpoint;

            return 1;

          }

        }

      } // for each neighbor
      
    }
  }

  return 0;

}

//////////////////////////////////////////////////////////////////////
// Create simple linear allocator for search nodes.

node_storage_t node_storage_create(size_t max_nodes) {

  node_storage_t storage;
  
  storage.start = malloc(max_nodes*sizeof(tree_node_t));
  
  if (!storage.start) {
    fprintf(stderr, "unable to allocate memory for node storage!\n");
    exit(1);
  }

  storage.capacity = max_nodes;
  storage.count = 0;

  return storage;
    
}

//////////////////////////////////////////////////////////////////////
// Allocate the next tree node.

tree_node_t* node_storage_alloc(node_storage_t* storage) {
  if (storage->count >= storage->capacity) {
    return NULL;
  }
  tree_node_t* rval = storage->start + storage->count;
  ++storage->count;
  return rval;
}

//////////////////////////////////////////////////////////////////////
// De-allocate a tree node -- note that we can only safely de-allocate
// the last node that was allocated. Of course we only check this with
// an assert, tho.

void node_storage_unalloc(node_storage_t* storage,
                          const tree_node_t* n) {

  assert( storage->count && n == storage->start + storage->count - 1 );
  --storage->count;

}

//////////////////////////////////////////////////////////////////////
// Free the memory allocated for this.

void node_storage_destroy(node_storage_t* storage) {
  free(storage->start);
}

//////////////////////////////////////////////////////////////////////
// Compare total cost for nodes, used by heap functions below.

int node_compare(const tree_node_t* a,
                 const tree_node_t* b) {

  double af = a->cost_to_come + a->cost_to_go;
  double bf = b->cost_to_come + b->cost_to_go;

  if (af != bf) {
    return af < bf ? -1 : 1;
  } else {
    return a < b ? -1 : 1;
  }

}

//////////////////////////////////////////////////////////////////////
// Create a binary heap to store the given # of nodes

queue_t heapq_create(size_t max_nodes) {
  queue_t rval;
  rval.heapq.start = malloc(sizeof(tree_node_t*) * max_nodes);
  if (!rval.heapq.start) {
    fprintf(stderr, "out of memory creating heapq!\n");
    exit(1);
  }
  rval.heapq.count = 0;
  rval.heapq.capacity = max_nodes;
  return rval;
}

//////////////////////////////////////////////////////////////////////
// Indexing macros for heaps

#define HEAPQ_PARENT_INDEX(i) (((i)-1)/2)
#define HEAPQ_LCHILD_INDEX(i) ((2*(i))+1)

//////////////////////////////////////////////////////////////////////
// For debugging, not used presently

int heapq_valid(const queue_t* q) {
  for (size_t i=1; i<q->heapq.count; ++i) {
    const tree_node_t* tc = q->heapq.start[i];
    const tree_node_t* tp = q->heapq.start[HEAPQ_PARENT_INDEX(i)];
    if (node_compare(tp, tc) > 0) {
      return 0;
    }
  }
  return 1;
}

//////////////////////////////////////////////////////////////////////
// Is heap queue empty?

int heapq_empty(const queue_t* q) {
  return q->heapq.count == 0;
}

//////////////////////////////////////////////////////////////////////
// Peek at the next item to be removed

const tree_node_t* heapq_peek(const queue_t* q) {
  assert(!heapq_empty(q));
  return q->heapq.start[0];
}

//////////////////////////////////////////////////////////////////////
// Enqueue a node onto the heap

void heapq_enqueue(queue_t* q, tree_node_t* node) {

  assert(q->heapq.count < q->heapq.capacity);

  size_t i = q->heapq.count++;
  size_t pi = HEAPQ_PARENT_INDEX(i);
  
  q->heapq.start[i] = node;
  
  while (i > 0 && node_compare(q->heapq.start[pi], q->heapq.start[i]) > 0) {
    tree_node_t* tmp = q->heapq.start[pi];
    q->heapq.start[pi] = q->heapq.start[i];
    q->heapq.start[i] = tmp;
    i = pi;
    pi = HEAPQ_PARENT_INDEX(i);
  }
                      
}

//////////////////////////////////////////////////////////////////////
// Helper function used for dequeueing

void _heapq_repair(queue_t* q, size_t i) {

  size_t li = HEAPQ_LCHILD_INDEX(i);
  size_t ri = li + 1;
  size_t smallest = i;

  if (li < q->heapq.count &&
      node_compare(q->heapq.start[i], q->heapq.start[li]) > 0) {
    smallest = li;
  }

  if (ri < q->heapq.count &&
      node_compare(q->heapq.start[smallest], q->heapq.start[ri]) > 0) {
    smallest = ri;
  }

  if (smallest != i){
    tree_node_t* tmp = q->heapq.start[i];
    q->heapq.start[i] = q->heapq.start[smallest];
    q->heapq.start[smallest] = tmp;
    _heapq_repair(q, smallest);
  }    

}

//////////////////////////////////////////////////////////////////////
// Pop a node off the heap

tree_node_t* heapq_deque(queue_t* q) {

  assert(!heapq_empty(q));

  tree_node_t* rval = q->heapq.start[0];
  --q->heapq.count;

  if (q->heapq.count) {
    q->heapq.start[0] = q->heapq.start[q->heapq.count];
    _heapq_repair(q, 0);
  }
  
  return rval;
  
}

//////////////////////////////////////////////////////////////////////
// Free memory allocated for heap

void heapq_destroy(queue_t* q) {
  free(q->heapq.start);
}

//////////////////////////////////////////////////////////////////////
// FIFO via flat array

queue_t fifo_create(size_t max_nodes) {
  queue_t rval;
  rval.fifo.start = malloc(sizeof(tree_node_t*) * max_nodes);
  if (!rval.fifo.start) {
    fprintf(stderr, "out of memory creating fifo!\n");
    exit(1);
  }
  rval.fifo.count = 0;
  rval.fifo.capacity = max_nodes;
  rval.fifo.next = 0;
  return rval;
}

//////////////////////////////////////////////////////////////////////
// Check if empty

int fifo_empty(const queue_t* q) {
  return q->fifo.next == q->fifo.count;
}

//////////////////////////////////////////////////////////////////////
// Push node into FIFO

void fifo_enqueue(queue_t* q, tree_node_t* n) {
  assert(q->fifo.count < q->fifo.capacity);
  q->fifo.start[q->fifo.count++] = n;
}

//////////////////////////////////////////////////////////////////////
// Peek at current FIFO node

const tree_node_t* fifo_peek(const queue_t* q) {
  assert(!fifo_empty(q));
  return q->fifo.start[q->fifo.next];
}

//////////////////////////////////////////////////////////////////////
// Dequeue node from FIFO

tree_node_t* fifo_deque(queue_t* q) {
  assert(!fifo_empty(q));
  return q->fifo.start[q->fifo.next++];
}

//////////////////////////////////////////////////////////////////////
// De-allocate storage for FIFO

void fifo_destroy(queue_t* q) {
  free(q->fifo.start);
}

//////////////////////////////////////////////////////////////////////
// Call this before calling the generic queue functions above.

void queue_setup() {

  if (g_options.search_astar_like) {

    queue_create = heapq_create;
    queue_enqueue = heapq_enqueue;
    queue_deque = heapq_deque;
    queue_destroy = heapq_destroy;
    queue_empty = heapq_empty;
    queue_peek = heapq_peek;

  } else {

    queue_create = fifo_create;
    queue_enqueue = fifo_enqueue;
    queue_deque = fifo_deque;
    queue_destroy = fifo_destroy;
    queue_empty = fifo_empty;
    queue_peek = fifo_peek;

  }

}

//////////////////////////////////////////////////////////////////////
// Create a node from the linear allocator. This does not properly set
// the cost to come and cost to go, those need to be finished later by
// node_update_costs.

tree_node_t* node_create(node_storage_t* storage,
                         tree_node_t* parent,
                         const game_info_t* info,
                         const game_state_t* state) {
  
  tree_node_t* rval = node_storage_alloc(storage);
  if (!rval) { return 0; }

  rval->parent = parent;
  rval->cost_to_come = 0;
  rval->cost_to_go = 0;

  memcpy(&(rval->state), state, sizeof(game_state_t));
  
  return rval;

}

//////////////////////////////////////////////////////////////////////
// Update the cost-to-come and cost-to-go for a node after a
// successful move has been made.

void node_update_costs(const game_info_t* info,
                       tree_node_t* n,
                       size_t action_cost) {

  // update cost to come
  if (n->parent) {
 
    n->cost_to_come = n->parent->cost_to_come + action_cost;

  } else {

    n->cost_to_come = 0;

  }
  
  n->cost_to_go = n->state.num_free;
  
}

//////////////////////////////////////////////////////////////////////
// Animate the solution by printing out boards in reverse order,
// following parent pointers back from solution to root.

void game_animate_solution(const game_info_t* info,
                           const tree_node_t* node) {

  if (node->parent) {
    game_animate_solution(info, node->parent);
  }

  printf("%s", unprint_board(info));
  game_print(info, &node->state);
  fflush(stdout);

  delay_seconds(0.1);
  
}

//////////////////////////////////////////////////////////////////////
// Return free if in bounds and unoccupied

int game_is_free(const game_info_t* info,
                 const game_state_t* state,
                 int x, int y) {

  return (coords_valid(info, x, y) &&
          state->cells[pos_from_coords(x, y)] == 0);
  
}

//////////////////////////////////////////////////////////////////////
// This is a helper function used by game_check_bottleneck below.

int game_check_chokepoint(const game_info_t* info,
                          const game_state_t* state,
                          int color, int dir, int endpoint, int n) {

  // Make the proposed move.
  game_state_t state_copy = *state;

  for (int i=0; i<n; ++i) {
    game_make_move(info, &state_copy, color, dir, endpoint);
  }

  // Build new region map
  uint8_t rmap[MAX_CELLS];
  size_t rcount = game_build_regions(info, &state_copy, rmap);

  // See if we are stranded 
  int result = game_regions_stranded(info, &state_copy, rcount, rmap,
                                     color, n+1);

  if (result) {
    return result;
  }
  
  return 0;

}

//////////////////////////////////////////////////////////////////////
// If a recent move has just 

int game_check_bottleneck(const game_info_t* info,
                          const game_state_t* state) {

  size_t color = state->last_move ? cell_get_color(state->last_move) : MAX_COLORS;

  if (color >= info->num_colors) { return 0; }
  int endpoint = (cell_get_type(state->last_move) == TYPE_INIT ? 0 : 1);

  pos_t pos = state->pos[color][endpoint];
  
  int x0, y0;
  pos_get_coords(pos, &x0, &y0);

  for (int dir=0; dir<4; ++dir) {

    int sign = (endpoint == 0 ? 1 : -1);
    int dx = sign * DIR_DELTA[dir][0];
    int dy = sign * DIR_DELTA[dir][1];

    int x1 = x0+dx;
    int y1 = y0+dy;

    if (game_is_free(info, state, x1, y1)) {
      for (int n=0; n<g_options.node_bottleneck_limit; ++n) {
        int x2 = x1+dx;
        int y2 = y1+dy;
        if (!game_is_free(info, state, x2, y2)) {
          int r = game_check_chokepoint(info, state, color, dir,
                                        endpoint, n+1);
          if (r) { return r; }
        }
        x1 = x2;
        y1 = y2;
      }
    }

    /*
    if (game_is_free(info, state, x1, y1) &&
        !game_is_free(info, state, x2, y2)) {


      int r = game_check_chokepoint(info, state, color, dir, endpoint, x1, y1);

      if (r) { return r; }
      
    }
    */
    
  }

  return 0;

}

//////////////////////////////////////////////////////////////////////
// Perform diagnostics on the given node

void game_diagnostics(const game_info_t* info,
                   const tree_node_t* node) {

  printf("\n###################################"
         "###################################\n\n");

  printf("node has cost to come %'g and cost to go %'g\n",
         node->cost_to_come, node->cost_to_go);

  if (node->state.last_move) {
    printf("last move was %s\n",
           color_cell_str(info, node->state.last_move));

  } else {
    printf("no moves yet?\n");
  }

  game_state_t state_copy = node->state;

  int forced = 1;
  
  while (forced) {

    printf("game state:\n\n");
    game_print(info, &state_copy);
    printf("\n");
    
    uint8_t rmap[MAX_CELLS];

    size_t rcount = game_build_regions(info, &state_copy, rmap);

    if (game_regions_stranded(info, &state_copy, rcount, rmap, MAX_COLORS, 1)) {
      printf("stranded -- state should be pruned!\n");
      printf("game regions:\n\n");
      game_print_regions(info, &state_copy, rmap);
      break;
    }

    if (game_regions_deadends(info, &state_copy, rcount, rmap)) {
      printf("dead-ended -- state should be pruned!\n");
      printf("game regions:\n\n");
      game_print_regions(info, &state_copy, rmap);
      break;
    }

    int r = game_check_bottleneck(info, &state_copy);
    if (r) {
      printf("chokepoint for ");
      for (size_t color=0; color<info->num_colors; ++color) {
        if (r & (1 << color)) {
          printf("%s", color_name_str(info, color));
        }
      }
      printf(" -- state should be pruned!\n");
      break;
    }

    int color, dir, endpoint;
    forced = game_find_forced(info, &state_copy,
                              &color, &dir, &endpoint);
    
    if (forced) {

      cell_t move = cell_create(TYPE_PATH, color, dir);

      printf("color %s is forced to move %s at end %d!\n",
             color_name_str(info, color),
             color_cell_str(info, move),
             endpoint);

      if (!game_can_move(info, &state_copy, color, dir, endpoint)) {
        printf("...but it is not allowed -- state should be pruned!\n");
        break;
      }

      game_make_move(info, &state_copy, color, dir, endpoint);
      
    }
    
  }
  
}

tree_node_t* game_validate_ff(const game_info_t* info,
                              tree_node_t* node,
                              node_storage_t* storage) {

  assert(node == storage->start+storage->count-1);


  //printf("validating node %p\n", node);
  //game_print(info, &node->state);
  
  const game_state_t* node_state = &node->state;
  
  if (g_options.node_check_stranded ||
      g_options.node_check_deadends) {
    
    uint8_t rmap[MAX_CELLS];
    size_t rcount = game_build_regions(info, node_state, rmap);
    
    if ( (g_options.node_check_stranded &&
          game_regions_stranded(info, node_state, rcount, rmap, MAX_COLORS, 1)) ||
         (g_options.node_check_deadends &&
          game_regions_deadends(info, node_state, rcount, rmap)) ) {

      goto unalloc_return_0;
            
    }

  }

  if (g_options.node_bottleneck_limit && 
      game_check_bottleneck(info, node_state)) {

    goto unalloc_return_0;
    
  }

  if (g_options.order_forced_first) {

    int color, dir, endpoint;
    
    if (game_find_forced(info, node_state,
                         &color, &dir, &endpoint)) {

      if (!game_can_move(info, node_state, color, dir, endpoint)) {
        goto unalloc_return_0;
      }
      
      tree_node_t* forced_child = node_create(storage, node, info,
                                              node_state);

      // if null, we ran out of memory and returning node is fine.
      
      if (forced_child) {

        game_make_move(info, &forced_child->state,
                       color, dir, endpoint);

        node_update_costs(info, forced_child, 0);
        forced_child = game_validate_ff(info, forced_child, storage);
      
        if (!forced_child) {
          goto unalloc_return_0;
        } else {
          return forced_child;
        }

      }
      
    }

  }

  //printf("node %p seems fine!\n", node);
  return node;

 unalloc_return_0:

  //printf("node %p failed to validate\n", node);
  assert(node == storage->start+storage->count-1);
  node_storage_unalloc(storage, node);
  return 0;
  
}

//////////////////////////////////////////////////////////////////////
// Peforms A* or BFS search

int game_search(const game_info_t* info,
                const game_state_t* init_state,
                const uint8_t* hint,
                double* elapsed_out,
                size_t* nodes_out) {

  size_t max_nodes = g_options.search_max_nodes;

  if (!max_nodes) {
    max_nodes = floor( g_options.search_max_mb * MEGABYTE /
                       sizeof(tree_node_t) );
  }

  node_storage_t storage = node_storage_create(max_nodes);

  tree_node_t* root = node_create(&storage, NULL, info, init_state);
  node_update_costs(info, root, 0);

  if (!g_options.display_quiet) {
    
    printf("will search up to %'zu nodes (%'.2f MB)\n",
           max_nodes, max_nodes*(double)sizeof(tree_node_t)/MEGABYTE);
  
    printf("heuristic at start is %'g\n\n",
           root->cost_to_go);

    game_print(info, init_state);

  }

  queue_t q = queue_create(max_nodes);

  int result = SEARCH_IN_PROGRESS;
  const tree_node_t* solution_node = NULL;

  double start = now();

  root = game_validate_ff(info, root, &storage);

  if (!root) {
    result = SEARCH_UNREACHABLE;
  } else {
    queue_enqueue(&q, root);
  }
  
  while (result == SEARCH_IN_PROGRESS) {

    if (queue_empty(&q)) {
      result = SEARCH_UNREACHABLE;
      break;
    }

    tree_node_t* n = queue_deque(&q);
    assert(n);

    game_state_t* parent_state = &n->state;
    
    int endpoint = 0;
    int color = game_next_move_color(info, parent_state, &endpoint);
    int hint_dir = -1;

    if (hint) {
      pos_t pos = parent_state->pos[color][endpoint];
      if (hint[pos] == color || hint[pos] >= info->num_colors) {
        for (int dir=0; dir<4; ++dir) {
          pos_t neighbor_pos = pos_offset_pos(info, pos, dir);
          if (neighbor_pos != INVALID_POS && 
              parent_state->cells[neighbor_pos] == 0 &&
              hint[neighbor_pos] == color) {
            hint_dir = dir;
            break;
          }
        }
      }
    }
      
    for (int dir=0; dir<4; ++dir) {

      if (hint_dir >= 0 && dir != hint_dir) { continue; }

      if (game_can_move(info, &n->state,
                        color, dir, endpoint)) {

        tree_node_t* child = node_create(&storage, n, info,
                                         parent_state);

        if (!child) {
          result = SEARCH_FULL;
          break;
          
        }

        size_t action_cost = game_make_move(info, &child->state,
                                            color, dir, endpoint);
        
        node_update_costs(info, child, action_cost);


        child = game_validate_ff(info, child, &storage);
        
        if (child) {

          const game_state_t* child_state = &child->state;
          
          if ( child_state->num_free == 0 && 
               ( queue_empty(&q) ||
                 node_compare(child, queue_peek(&q) ) <= 0) ) {          
          
            result = SEARCH_SUCCESS;
            solution_node = child;
          
            break;
      
          }
          
          queue_enqueue(&q, child);
        }

      } // if can move 

    } // for each dir

  } // while search active

  double elapsed = now() - start;
  if (elapsed_out) { *elapsed_out = elapsed; }
  if (nodes_out)   { *nodes_out = storage.count; }
  

  if (!g_options.display_quiet) {
  
    if (result == SEARCH_SUCCESS) {
      assert(solution_node);
      if (!g_options.display_animate) {
        printf("\n");
        game_print(info, &solution_node->state);
      } else {
        delay_seconds(1.0);
        game_animate_solution(info, solution_node);
        delay_seconds(1.0);
      }
    } 

    double storage_mb = (storage.count * (double)sizeof(tree_node_t) / MEGABYTE);

    printf("\nsearch %s after %'.3f seconds and %'zu nodes (%'.2f MB)\n",
           SEARCH_RESULT_STRINGS[result],
           elapsed,
           storage.count, storage_mb);

    if (result == SEARCH_SUCCESS) {
    
      assert(solution_node);

      printf("final cost to come=%'g, cost to go=%'g\n",
             solution_node->cost_to_come,
             solution_node->cost_to_go);

    } else if (result == SEARCH_FULL && g_options.display_diagnostics) {

      printf("here's the lowest cost thing on the queue:\n");

      game_diagnostics(info, queue_peek(&q));

      printf("\nand here's the last node allocated:\n");

      game_diagnostics(info, storage.start+storage.count-1);
      
    }

  }

  node_storage_destroy(&storage);
  queue_destroy(&q);

  return result;
  
}

//////////////////////////////////////////////////////////////////////
// Command line usage

void usage(FILE* fp, int exitcode) {

  fprintf(fp,
          "usage: flow_solver [ OPTIONS ] [ -H HINT1.txt ] [ -o ORDER1 ] BOARD1.txt\n"
          "                   [ [ -H HINT2.txt ] [ -o ORDER2 ] BOARD2.txt [ ... ] ]\n\n"
          "Display options:\n\n"
          "  -q, --quiet             Reduce output\n"
          "  -D, --diagnostics       Print diagnostics when search unsuccessful\n"
          "  -A, --no-animation      Disable animating solution\n"
          "  -F, --fast              Speed up animation 4x\n"
#ifndef _WIN32          
          "  -C, --color             Force use of ANSI color\n"
#endif
          "\n"
          "Node evaluation options:\n\n"
          "  -t, --touch             Disable path self-touch test\n"
          "  -s, --stranded          Disable stranded checking\n"
          "  -d, --deadends          Disable dead-end checking\n"
          "  -b, --bottlenecks N     Set bottleneck limit check (default %d)\n"
          "  -e, --no-explore        Penalize exploring away from walls\n"
          "\n"
          "Color ordering options:\n\n"
          "  -a, --no-autosort       Disable auto-sort of color order\n"
          "  -r, --randomize         Shuffle order of colors before solving\n"
          "  -f, --forced            Disable ordering forced moved first\n"
          "  -c, --constrained       Disable order by most constrained\n"
          "\n"
          "Search options:\n\n"
          "  -O, --no-outside-in     Disable outside-in searching\n"
          "  -B, --bfs               Run breadth-first search\n"
          "  -n, --max-nodes N       Restrict storage to N nodes\n"
          "  -m, --max-storage N     Restrict storage to N MB (default %'g)\n"
          "\n"
          "Options affecting the next input file:\n\n"
          "  -o, --order ORDER       Set color order on command line\n"
          "  -H, --hint HINTFILE     Provide hint for previous board.\n"
          "\n"
          "Help:\n\n"
          "  -h, --help              See this help text\n\n",
          g_options.node_bottleneck_limit,
          g_options.search_max_mb);

  exit(exitcode);
  
}

//////////////////////////////////////////////////////////////////////
// Check file exists

int exists(const char* fn) {

  FILE* fp = fopen(fn, "r");
  
  if (fp) {
    fclose(fp);
    return 1;
  } else {
    return 0;
  }

}

//////////////////////////////////////////////////////////////////////
//

const char* get_argument(int argc, char** argv, int* i) {

  assert(*i < argc);
  
  if ((*i)+1 == argc) {
    fprintf(stderr, "%s needs argument\n", argv[*i]);
    usage(stderr, 1);
  }

  return argv[++(*i)];
  
  
}

//////////////////////////////////////////////////////////////////////
// Parse command-line options

size_t parse_options(int argc, char** argv,
                     const char** input_files,
                     const char** user_orders,
                     const char** hint_files) {
  
  size_t num_inputs = 0;

  if (argc < 2) {
    fprintf(stderr, "not enough args!\n\n");
    usage(stderr, 1);
  }

  typedef struct flag_options_struct {
    int short_char;
    const char* long_string;
    int* dst_flag;
    int dst_value;
  } flag_options_t;

  flag_options_t options[] = {
    { 'q', "quiet",         &g_options.display_quiet, 1 },
    { 'D', "diagnostics",   &g_options.display_diagnostics, 1 },
    { 'A', "animation",     &g_options.display_animate, 0 },
    { 'F', "fast",          &g_options.display_fast, 1 },
#ifndef _WIN32    
    { 'C', "color",         &g_options.display_color, 1 },
#endif
    { 't', "touch",         &g_options.node_check_touch, 0 },
    { 's', "stranded",      &g_options.node_check_stranded, 0 },
    { 'd', "deadends",      &g_options.node_check_deadends, 0 },
    { 'b', "bottlenecks",   0, 0 },
    { 'e', "no-explore",    &g_options.node_penalize_exploration, 1 },
    { 'a', "no-autosort",   &g_options.order_autosort_colors, 0 },
    { 'o', "order",         0, 0 },
    { 'r', "randomize",     &g_options.order_random, 1 },
    { 'f', "forced",        &g_options.order_forced_first, 0 },
    { 'c', "constrained",   &g_options.order_most_constrained, 0 },
    { 'O', "no-outside-in", &g_options.search_outside_in, 0 },
    { 'B', "bfs",           &g_options.search_astar_like, 0 },
    { 'n', "max-nodes",     0, 0 },
    { 'm', "max-storage",   0, 0 },
    { 'H', "hint",          0, 0 },
    { 'h', "help",          0, 0 },
    { 0, 0, 0, 0 }
  };

  for (int i=1; i<argc; ++i) {
    
    const char* opt = argv[i];
    int match_id = -1;

    for (int k=0; options[k].short_char; ++k) {

      if (options[k].short_char > 0) {
        char cur_short[3] = "-?";
        cur_short[1] = options[k].short_char;
        if (!strcmp(opt, cur_short)) {
          match_id = k;
          break;
        }
      }

      if (options[k].long_string) {
        char cur_long[1024];
        snprintf(cur_long, 1024, "--%s", options[k].long_string);
        if (!strcmp(opt, cur_long)) {
          match_id = k;
          break;
        }
      }

    }

    if (match_id >= 0) {

      int match_short_char = options[match_id].short_char;

      if (options[match_id].dst_flag) {
        
        *options[match_id].dst_flag = options[match_id].dst_value;

      } else if (match_short_char == 'b') {
                
        opt = get_argument(argc, argv, &i);
      
        char* endptr;
        g_options.node_bottleneck_limit = strtol(opt, &endptr, 10);
      
        if (!endptr || *endptr) {
          fprintf(stderr, "error parsing bottleneck limit %s on command line!\n\n", opt);
          exit(1);
        }

      } else if (match_short_char == 'n') {

        opt = get_argument(argc, argv, &i);
      
        char* endptr;
        g_options.search_max_nodes = strtol(opt, &endptr, 10);
      
        if (!endptr || *endptr) {
          fprintf(stderr, "error parsing max nodes %s on command line!\n\n", opt);
          exit(1);
        }

      } else if (match_short_char == 'm') {

        opt = get_argument(argc, argv, &i);
        
        char* endptr;
        g_options.search_max_mb = strtod(opt, &endptr);
        
        if (!endptr || *endptr || g_options.search_max_mb <= 0) {
          fprintf(stderr, "error parsing max storage %s on command line!\n\n", opt);
          exit(1);
        }
        
      } else if (match_short_char == 'H') {

        opt = get_argument(argc, argv, &i);
      
        if (!exists(opt)) {
          fprintf(stderr, "error opening %s\n", opt);
          exit(1);
        }
      
        hint_files[num_inputs] = opt;

      } else if (match_short_char == 'o') {

        user_orders[num_inputs] = get_argument(argc, argv, &i);
        
      } else { // should not happen

        fprintf(stderr, "unrecognized option: %s\n\n", opt);
        usage(stderr, 1);

      }

    } else if (exists(opt)) {

      input_files[num_inputs++] = opt;

    } else {

      fprintf(stderr, "unrecognized option: %s\n\n", opt);
      usage(stderr, 1);
      
    }
    
  }

  if (!num_inputs) {
    fprintf(stderr, "no input files\n\n");
    exit(1);
  } else if (user_orders[num_inputs]) {
    fprintf(stderr, "order specified *after* last input file!\n\n");
    exit(1);
  } else if (hint_files[num_inputs]) {
    fprintf(stderr, "hint file specified *after* last input file!\n\n");
    exit(1);
  }

  return num_inputs;

}

//////////////////////////////////////////////////////////////////////
// Main function

int main(int argc, char** argv) {

  setlocale(LC_NUMERIC, "");

  g_options.display_quiet = 0;
  g_options.display_diagnostics = 0;
  g_options.display_animate = 1;
  g_options.display_color = terminal_has_color();
  g_options.display_fast = 0;
  
  g_options.node_check_touch = 1;
  g_options.node_check_stranded = 1;
  g_options.node_check_deadends = 1;
  g_options.node_bottleneck_limit = 3;
  g_options.node_penalize_exploration = 0;

  g_options.order_autosort_colors = 1;
  g_options.order_most_constrained = 1;
  g_options.order_forced_first = 1;

  g_options.search_outside_in = 1;
  g_options.search_astar_like = 1;
  g_options.search_max_nodes = 0;
  g_options.search_max_mb = 1600; // damn you, jumbo_14x14_19.txt!
  g_options.search_max_endpoint = 1;

  const char* input_files[argc];
  const char* user_orders[argc];
  const char* hint_files[argc];

  memset(input_files, 0, sizeof(input_files));
  memset(user_orders, 0, sizeof(user_orders));
  memset(hint_files,  0, sizeof(hint_files));
  
  size_t num_inputs = parse_options(argc, argv,
                                    input_files,
                                    user_orders,
                                    hint_files);

  queue_setup();

  game_info_t  info;
  game_state_t state;
  pos_t hint[MAX_CELLS];
  
  int max_width = 11;

  for (size_t i=0; i<num_inputs; ++i) {
    int l = strlen(input_files[i]);
    if (l > max_width) { max_width = l; }
  }

  int boards = 0;
  double total_elapsed[3] = { 0, 0, 0 };
  size_t total_nodes[3]   = { 0, 0, 0 };
  int    total_count[3]   = { 0, 0, 0 };
  
  for (size_t i=0; i<num_inputs; ++i) {

    const char* input_file = input_files[i];
    const char* hint_file = hint_files[i];
    const char* user_order = user_orders[i];
  
    if (game_read(input_file, &info, &state)) {

      if (boards++ && !g_options.display_quiet) {
        printf("\n***********************************"
               "***********************************\n\n");
      }

      if (hint_file) {
        if (!game_read_hint(&info, &state, hint_file, hint)) {
          hint_file = 0;
        } 
      }
      
      if (!g_options.display_quiet) {
        printf("read %zux%zu board with %zu colors from %s\n",
               info.size, info.size, info.num_colors, input_file);
        if (hint_file) {
          printf("read hint file from %s\n", hint_file);
        }
        printf("\n");
      }

      game_order_colors(&info, &state, user_order);

      double elapsed;
      size_t nodes;

      int result = game_search(&info, &state, hint_file ? hint : 0,
                               &elapsed, &nodes);

      assert( result >= 0 && result < 3 );

      total_elapsed[result] += elapsed;
      total_nodes[result] += nodes;
      total_count[result] += 1;

      if (g_options.display_quiet) {
        
        printf("%*s %c %'12.3f %'12zu\n",
               max_width, input_file,
               SEARCH_RESULT_CHARS[result],
               elapsed, nodes);

      }
      
    }

  }

  if (boards > 1) {

    double overall_elapsed = 0;
    size_t overall_nodes = 0;
    int types = 0;
    
    for (int i=0; i<3; ++i) {
      overall_elapsed += total_elapsed[i];
      overall_nodes += total_nodes[i];
      if (total_nodes[i]) { ++types; }
    }

    if (!g_options.display_quiet) {

      printf("\n***********************************"
             "***********************************\n\n");

      for (int i=0; i<3; ++i) {
        if (total_count[i]) {
          printf("%'d %s searches took a total of %'.3f seconds and %'zu nodes\n",
                 total_count[i], SEARCH_RESULT_STRINGS[i],
                 total_elapsed[i], total_nodes[i]);
        }
      }

      if (types > 1) {
        printf("\n");
        printf("overall, %'d searches took a total of %'.3f seconds and %'zu nodes\n",
               boards, overall_elapsed, overall_nodes);
      }
      
    } else {
      
      printf("\n");
      for (int i=0; i<3; ++i) {
        if (total_count[i]) {
          printf("%*s%3d total %c %'12.3f %'12zu\n",
                 max_width-9, "",
                 total_count[i],
                 SEARCH_RESULT_CHARS[i],
                 total_elapsed[i],
                 total_nodes[i]);
        }
      }

      if (types > 1) {
        printf("\n");
        printf("%*s%3d overall %'12.3f %'12zu\n",
               max_width-9, "",
               boards,
               overall_elapsed,
               overall_nodes);
      }
      
    }
    
  }
  
    
  return 0;
  
}

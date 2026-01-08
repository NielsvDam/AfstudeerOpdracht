#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// matrix properties
#define MATRIX_ROWS 1120
#define MATRIX_COLUMNS 720
#define MATRIX_ORIGIN_X MATRIX_ROWS / 2
#define MATRIX_ORIGIN_Y MATRIX_COLUMNS / 2
#define CELL_SIZE 0.5 // in millimeters

// algorithm specific settings
#define BLOCK_HEIGHT_INDICATION_TOLERANCE 8 // Was 5mm, now 8mm
#define BLOCK_SIZE 18.9                     // 18.9mm
#define SINGLE_BLOCK_DETECTION_TOLERANCE 8  // 8mm. Tolerance to determine if a detected square is the size of a single block

#define SURFACE_COLLECT_MAX_DIFF 3            // 3mm
#define SURFACE_COLLECT_HISTORY_WINDOW_SIZE 4 // 4 cells

#endif /* CONSTANTS_HPP */
world {}

wall (world){} 

# SHELF
#########################

shelf_side_left (wall):{ X:"t(-0.3 0.5 1.025) d(0 0 0 1)" , shape:box, size: [0.03 0.40 0.85 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

shelf_side_right (wall):{ X:"t(0.3 0.5 1.025) d(0 0 0 1)" , shape:box, size: [0.03 0.40 0.85 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

shelf_side_bottom (wall):{ X:"t(0 0.5 0.675) d(0 0 0 1)" , shape:box, size: [0.60 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

shelf_side_top (wall):{ X:"t(0 0.5 1.45) d(0 0 0 1)" , shape:box, size: [0.60 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

shelf_side_mid (wall):{ X:"t(0 0.5 1.1) d(0 0 0 1)" , shape:box, size: [0.60 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

shelf_side_mid_2 (wall):{ X:"t(0 0.5 0.8) d(0 0 0 1)" , shape:box, size: [0.60 0.40 0.03 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

shelf_back (wall):{ X:"t(0 0.7 1.025) d(0 0 0 1)" , shape:box, size: [0.60 0.03 0.85 0.5], color: [0.996 0.478 0.211], mass: 1,contact:-1}

# BOOKS
#########################

book_1 (wall): { X:"t(-0.25 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_2 (wall): { X:"t(-0.19 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_3 (wall): { X:"t(-0.13 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_4 (wall): { X:"t(-0.07 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_5 (wall): { X:"t(-0.01 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_6 (wall): { X:"t(0.05 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book (world): { X:"t(0 0.4 0.95) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [0 1 1], mass: .1,contact:1,
#book (world): { X:"t(-0.16 0.4 0.95) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [0 1 1], mass: .1,contact:1,
joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]}
book_7 (wall): { X:"t(0.23 0.58 1.215) d(0 0 0 1)" , shape:box, size: [0.06 0.20 0.20 0.5], color: [1 0 0], mass: .1,contact:-1}
book_obstacle (wall): { X:"t(-0.12 0.4 0.915) d(0 0 0 1)" , shape:box, size: [0.14 0.03 0.20 0.5], color: [1 1 0], mass: .1,contact:1}

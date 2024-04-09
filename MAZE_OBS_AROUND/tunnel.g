world {}

wall (world){} 




#lock_top (wall):{ X:"t(-0.3 0.25 0.765) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

#lock_side (wall):{ X:"t(-0.3 0.19 0.68) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

#lock_side2 (wall):{ X:"t(-0.3 0.31 0.68) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}



bin_side (wall):{ X:"t(-0.1 0.1 0.68) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}

bin_side2 (wall):{ X:"t(-0.1 0.7 0.68) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}

bin_side3 (wall):{ X:"t(0.4 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}

bin_side4 (wall):{ X:"t(-0.6 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}

bin_side5 (wall):{ X:"t(-0.2 0.45 0.68) d(0 0 0 1)" , shape:box, size: [0.15 0.25 0.1 0.5], color: [0.3 1 0], mass: 10,contact:1}


world {}

wall (world){} 

lock_top (wall):{ X:"t(-0.9 0.25 0.745) d(0 0 0 1)" , shape:box, size: [0.9 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}
lock_side (wall):{ X:"t(-0.9 0.19 0.66) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}
lock_side2 (wall):{ X:"t(-0.9 0.31 0.66) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}
lock1_top (wall):{ X:"t(-0.2 0.25 0.745) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}
lock1_side (wall):{ X:"t(-0.2 0.19 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}
lock1_side2 (wall):{ X:"t(-0.2 0.31 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

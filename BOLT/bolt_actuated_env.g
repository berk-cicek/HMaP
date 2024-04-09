world {}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .54)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact:1,
}

box (table){
  shape:ssBox, Q: "t(-0.32 0.25 .12) d(0 0 0 1)", size:[1.0 .1 .1 .04], mass: .1, contact:1,
  joint:free, limits: [-10, 10, -10, 10, 0.11, 0.13, -1.7, 1.7, -1.7, 1.7, -0.2, 0.2, -0.2, 0.2]
}

head (box){
  shape:ssBox, Q: "t(-0.05 0.07 .00) d(0 0 0 1)", size:[.02 .2 .01 .02], color:[0 1 1], mass: .1, contact:1
}

lock_top (wall):{ X:"t(-0.6 0.25 0.748) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.6 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.6 0.35 0.66) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], mass: .1,contact:-1}

lock0_side (wall):{ X:"t(-0.4 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], mass: .1,contact:-1}

lock1_side (wall):{ X:"t(-0.2 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], mass: .1,contact:-1}

lock1_side2 (wall):{ X:"t(-0.2 0.35 0.66) d(0 0 0 1)" , shape:box, size: [0.1 0.03 0.14 0.5], mass: .1,contact:-1}

lock1_top1 (wall):{ X:"t(-0.2 0.16 0.748) d(0 0 0 1)" , shape:box, size: [0.1 0.04 0.03 0.5], mass: .1,contact:-1}

lock1_top2 (wall):{ X:"t(-0.2 0.34 0.748) d(0 0 0 1)" , shape:box, size: [0.1 0.05 0.03 0.5], mass: .1,contact:-1}



lock02_side (wall):{ X:"t(-0.0 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], mass: .1,contact:-1}

lock2_top (wall):{ X:"t(0.1 0.25 0.748) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], mass: .1,contact:-1}

lock2_side (wall):{ X:"t(0.1 0.15 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], mass: .1,contact:-1}

lock2_side2 (wall):{ X:"t(0.1 0.35 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], mass: .1,contact:-1}

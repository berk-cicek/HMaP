world {}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .00)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact
}

lock_top (wall):{ X:"t(-0.9 0.25 0.165) d(0 0 0 1)" , shape:box, size: [0.9 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.9 0.19 0.08) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.9 0.31 0.08) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_top (wall):{ X:"t(-0.2 0.25 0.165) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side (wall):{ X:"t(-0.2 0.19 0.08) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side2 (wall):{ X:"t(-0.2 0.31 0.08) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}


box (table){
  shape:ssBox, Q: "t(-0.64 0.25 .09001) d(0 0 0 1)", size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]
}



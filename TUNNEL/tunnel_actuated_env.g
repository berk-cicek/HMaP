world {}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .60)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact
}

lock_top (wall):{ X:"t(-0.9 0.25 0.765) d(0 0 0 1)" , shape:box, size: [0.9 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_side (wall):{ X:"t(-0.9 0.19 0.68) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_side2 (wall):{ X:"t(-0.9 0.31 0.68) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_top (wall):{ X:"t(-0.2 0.25 0.765) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side (wall):{ X:"t(-0.2 0.19 0.68) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side2 (wall):{ X:"t(-0.2 0.31 0.68) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}


final:{ X:"t(0.4 0.25 0.7) d(0 0 0 1)" , shape: marker, size: [.01]}
#middle:{ X:"t(0 0.25 0.7) d(0 0 0 1)" , shape: marker, size: [.01]}
middle:{ X:"t(0 0.25 0.7) d(0 0 0 1)" , shape: marker, size: [.01]}

wp1:{ X:"t(-0.58 0.25 0.7) d(0 0 0 1)" , shape: marker, size: [.1]}
wp2:{ X:"t(-0.68 0.25 0.7) d(0 0 0 1)" , shape: marker, size: [.1]}
#cp1:{ X:"t(-0.38 0.25 0.73) d(0 0 0 1)" , shape: marker, size: [.1]}
#cpo (box):{ X:"t(0.26 0.0 0.03) d(0 0 0 1)" , shape: marker, size: [.1]}
cp2:{ X:"t(-0.04 0.25 0.73) d(0 0 0 1)" , shape: marker, size: [.1]}

box (world){
  shape:ssBox, Q: "t(-0.64 0.25 .69001) d(0 0 0 1)", size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]
}



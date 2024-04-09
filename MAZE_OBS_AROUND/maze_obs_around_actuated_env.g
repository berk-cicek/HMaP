world {}

wall (world){} 

bin_side (wall):{ X:"t(-0.1 0.1 0.68) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}
bin_side (wall):{ X:"t(-0.1 0.1 0.68) d(0 0 0 1)" , shape:box, size: [1 0.18 0.1 0.5], color: [0.3 1 0 0.2], mass: 10,contact:-1}

bin_side2 (wall):{ X:"t(-0.1 0.7 0.68) d(0 0 0 1)" , shape:box, size: [1 0.09 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}
bin_side2 (wall):{ X:"t(-0.1 0.7 0.68) d(0 0 0 1)" , shape:box, size: [1 0.25 0.1 0.5], color: [0.3 1 0 0.2], mass: 10,contact:-1}

bin_side3 (wall):{ X:"t(0.4 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}

bin_side4 (wall):{ X:"t(-0.6 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.09 0.6 0.1 0.5], color: [0.3 1 0], mass: 10,contact:-1}
bin_side4 (wall):{ X:"t(-0.6 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.18 0.6 0.1 0.5], color: [0.3 1 0 0.2], mass: 10,contact:-1}

bin_side5 (wall):{ X:"t(-0.2 0.45 0.68) d(0 0 0 1)" , shape:box, size: [0.15 0.25 0.1 0.5], color: [0.3 1 0], mass: 10,contact:1}
bin_side5 (wall):{ X:"t(-0.2 0.45 0.68) d(0 0 0 1)" , shape:box, size: [0.27 0.34 0.1 0.5], color: [0.3 1 0 0.2], mass: 10,contact:1}

floor (world){
  Q: "t(0.0 0.0 0.66) d(0 0 0 1)" 
}

box (floor){
  shape:ssCylinder, Q: "t(0.0 0.4 .00001) d(0 0 0 1)",size:[.001 .03 .01], color:[0 1 1], mass: .1, contact:1,
  joint:transXYPhi, limits: [-10,10,-10,10,-1,1]
}



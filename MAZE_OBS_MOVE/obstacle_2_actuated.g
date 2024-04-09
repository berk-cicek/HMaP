world {}

table(world): {
 shape: ssBox, Q: "t(0 0. .6)", size: [2.5, 2.5, .1, .02], color: [.3, .3, .3],
 contact: 0, logical: { }
}

floor (world){
  Q: "t(0.0 0.0 0.68) d(0 0 0 1)" 
}

obstacle (floor){
  shape:box, Q: "t(-0.3 0.4 .00001) d(0 0 0 1)", size:[0.04 .04 .04 .04], color:[0 1 0], mass: .1, contact:1,
  joint:transXYPhi, limits: [-10,10,-10,10,-1,1]
}

wall (world){} 

bin_side (wall):{ X:"t(-0.2 0.2 0.68) d(0 0 0 1)" , shape:box, size: [0.8 0.03 0.14 0.5], color: [0.3 1 0], mass: .1,contact:-1}

bin_side2 (wall):{ X:"t(-0.2 0.6 0.68) d(0 0 0 1)" , shape:box, size: [0.8 0.03 0.14 0.5], color: [0.3 1 0], mass: .1,contact:-1}

bin_side3 (wall):{ X:"t(0.2 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [0.3 1 0], mass: .1,contact:-1}

bin_side4 (wall):{ X:"t(-0.6 0.4 0.68) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [0.3 1 0], mass: .1,contact:-1}
world {}


floor (world){
  Q: "t(0.0 0.0 0.66) d(0 0 0 1)" 
}

box (floor){
  shape:ssCylinder, Q: "t(0.0 0.4 .00001) d(0 0 0 1)",size:[.001 .03 .01], color:[0 1 1], mass: .1, contact:1,
  joint:transXYPhi, limits: [-10,10,-10,10,-1,1]
}



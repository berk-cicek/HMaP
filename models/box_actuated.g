world {}

box (world){
  shape:ssBox, Q: "t(-0.64 0.25 .04) d(0 0 0 1)", size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]
}


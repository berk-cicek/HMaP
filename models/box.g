world {}

table (world){
    shape:ssBox, Q: "t(0 0. .0)", size:[4. 4. .1 .02], color:[0.3 0.3 0.3], contact
}

box (table){
  shape:ssBox, Q: "t(-0.64 0.25 .09) d(0 0 0 1)", size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:rigid
}
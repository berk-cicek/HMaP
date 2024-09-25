world {}

table (world){
    shape:ssBox, Q: "t(0 0. .05)", size:[4. 4. .1 .02], color:[.30 .30 .30] , mass: 1, contact:1
}

tunnel (world){
  shape:marker, Q: "t(0 0. .00)", size:[0.01]
}

tunnel_top (tunnel):{ Q:"t(-0.9 0.25 0.215) d(0 0 0 1)" , shape:box, size: [0.9 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_side (tunnel):{ Q:"t(-0.9 0.19 0.13) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_side2 (tunnel):{ Q:"t(-0.9 0.31 0.13) d(0 0 0 1)" , shape:box, size: [0.9 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_top_2 (tunnel):{ Q:"t(-0.2 0.25 0.215) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_side_2 (tunnel):{ Q:"t(-0.2 0.19 0.13) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_side2_2 (tunnel):{ Q:"t(-0.2 0.31 0.13) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}

box (world){
  X: "t(0 0 0) d(0 0 0 1)", shape:ssBox,  size:[1.0 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:free, limits: [-10,10,-10,10,-10,10,-4,4,-4,4,-4,4,-4,4]
}



world {}

table (world){
    shape:ssBox, Q: "t(0 0. .0)", size:[4. 4. .1 .02], color:[.3 .3 .3], mass: 1,  contact:1
}

stick (world){
  shape:ssBox, Q: "t(0.34 0.25 .09) d(90 0 0 1)", size:[0.4 .02 .025 .04], color:[0 1 0], mass: .1, contact:1,
  joint:rigid
}

sphere (world){
  shape:box, Q: "t(0.5 0.25 .10) d(90 0 0 1)", size:[0.08 .08 .08 .04], color:[1 0 1], mass: .1, contact:1,
  joint:rigid
}

cube (world){
  shape:box, Q: "t(0.1 0.5 .10) d(90 0 0 1)", size:[0.08 .08 .08 .04], color:[0 0 1], mass: .1, contact:1,
  joint:rigid
}

tunnel (world){
  shape:marker, Q: "t(0 0. .00)", size:[0.01]
}

tunnel_top (tunnel):{ Q:"t(-0.2 0.27 0.165) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_side (tunnel):{ Q:"t(-0.2 0.21 0.08) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
tunnel_side2 (tunnel):{ Q:"t(-0.2 0.33 0.08) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0.996 0.478 0.211], mass: 5,contact:-1}
box (tunnel){
  shape:ssBox, Q: "t(-0.2 0.27 .09001) d(180 0 0 1)", size:[0.3 .04 .05 .04], color:[0 1 1], mass: .1, contact:1,
  joint:rigid
}

bin (world){
  shape:marker, Q: "t(0 0 0)", size:[0.01]
}
bin_side (bin):{ Q:"t(-0.7 -0.6 0.08) d(0 0 0 1)" , shape:box, size: [0.4 0.03 0.14 0.5], color: [1 0.5 0], mass: 5,contact:-1}
bin_side2 (bin):{ Q:"t(-0.7 -0.2 0.08) d(0 0 0 1)" , shape:box, size: [0.4 0.03 0.14 0.5], color: [1 0.5 0], mass: 5,contact:-1}
bin_side3 (bin):{ Q:"t(-0.5 -0.4 0.08) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: 5,contact:-1}
bin_side4 (bin):{ Q:"t(-0.9 -0.4 0.08) d(0 0 0 1)" , shape:box, size: [0.03 0.4 0.14 0.5], color: [1 0.5 0], mass: 5,contact:-1}

contact_point(box){
  shape:marker, Q:"t(0 0 .025) d(0 0 0 1)", size: [0.1]
}



cam_frame_0: {pose: [-0.2, 0.1, 2.6, 0.0007963, -0.9999997, 0, 0], shape: marker, size: [0.1]}
cam_frame_1: {pose: [0.0, 0.1, 2.6,  0.0007963, -0.9999997, 0, 0], shape: marker, size: [0.1]}
cam_frame_2: {pose: [0.2, 0.1, 2.6, 0.0007963, -0.9999997, 0, 0], shape: marker, size: [0.1]}


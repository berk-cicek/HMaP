world {}

x{}

y{}

wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .0)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact:0}
shelf_side_right_low (wall):{ X:"t(0.3 0.5 0.3) d(0 0 0 1)" , shape:box, size: [0.03 0.40 0.6 0.5], color: [0.996 0.478 0.211], mass: 1,contact:0}
shelf_side_left_low (wall):{ X:"t(-0.3 0.5 0.3) d(0 0 0 1)" , shape:box, size: [0.03 0.40 0.6 0.5], color: [0.996 0.478 0.211], mass: 1,contact:0}
shelf_back (wall):{ X:"t(0 0.7 0.3) d(0 0 0 1)" , shape:box, size: [0.60 0.03 0.6 0.5], color: [0.996 0.478 0.211], mass: 1,contact:0}

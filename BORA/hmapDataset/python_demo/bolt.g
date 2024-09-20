world {}
lock_part: { },
wall (world){} 

table (world){
    shape:ssBox, Q: "t(0 0. .54)", size:[4. 4. .1 .02], color:[.3 .3 .3], contact
}

camera_1(world): {
 Q: "t(0.0 0.0 1.6) d(180 0 1 0) d(345 1 0 0)",
 shape: marker, size: [.1],
 focalLength: 0.8, width: 224, height: 224, zRange: [.5, 100]
}

camera_2(world): {
 Q: "t(0.6 0.6 1.6) d(180 0 1 0) d(30 1 0 0) d(30 0 1 0)",
 shape: marker, size: [.1],
 focalLength: 0.8, width: 224, height: 224, zRange: [.5, 100]
}

camera_3(world): {
 Q: "t(-.6 0.6 1.6) d(180 0 1 0) d(30 1 0 0) d(330 0 1 0)",
 shape: marker, size: [.1],
 focalLength: 0.8, width: 224, height: 224, zRange: [.5, 100]
}

Include: <../rai-robotModels/panda/panda.g>
Edit panda_link0{ X: "t(-0.2 0.9 .65) d(-90 0 0 1)" }
Edit panda_finger_joint1 { joint_active: False }

box (table){
  shape:ssBox, Q: "t(-0.35 0.25 .20) d(0 0 0 1)", size:[1.0 .05 .05 .04], color:[1 0 0], mass: .1, contact:1,
  joint:rigid
}

head_body (box){
  shape:ssBox, Q: "t(-0.05 0.11 .00) d(0 0 0 1)", size:[.02 .14 .01 .02], color:[0 0 1], mass: .1, contact:1,
  joint:rigid
}

head_tip (head_body){
  shape:ssBox, Q: "t(0 0.09 .00) d(0 0 0 1)", size:[.02 .02 .01 .02], color:[0 1 0], mass: .1, contact:1,
  joint:rigid
}

lock_part_1 (box){Q:"t(.33 -.25 -.66) d(0 0 0 1)"}

lock1_top (lock_part_1):{ Q:"t(-0.6 0.25 0.745) d(0 0 0 1)" , shape:box, size: [0.3 0.14 0.03 0.5], color: [1 0 0], mass: .1,contact:-1}

lock1_side_l (lock_part_1):{ Q:"t(-0.6 0.19 0.66) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock1_side_r (lock_part_1):{ Q:"t(-0.6 0.31 0.66) d(0 0 0 1)" , shape:box, size: [0.3 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_part_2 (lock_part_1) {}

lock2_top_l (lock_part_2):{ Q:"t(-0.2 0.2 0.745) d(0 0 0 1)" , shape:box, size: [0.2 0.04 0.03 0.5], color: [1 1 0], mass: .1,contact:-1}

lock2_top_r (lock_part_2):{ Q:"t(-0.22 0.3 0.745) d(0 0 0 1)" , shape:box, size: [0.20 0.05 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock2_side_l (lock_part_2):{ Q:"t(-0.2 0.19 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock2_side_r (lock_part_2):{ Q:"t(-0.22 0.31 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_part_3 (lock_part_2) {}

lock3_top (lock_part_3):{ Q:"t(0.1 0.25 0.745) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock3_side_l (lock_part_3):{ Q:"t(0.1 0.19 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock3_side_r (lock_part_3):{ Q:"t(0.1 0.31 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock_part_4 (lock_part_3) {Q:"t(0 0 0) d(0 0 0 1)"}

lock4_top (lock_part_4):{ Q:"t(0.4 0.25 0.745) d(0 0 0 1)" , shape:box, size: [0.2 0.14 0.03 0.5], color: [0 0 0], mass: .1,contact:-1}

lock4_side_l (lock_part_4):{ Q:"t(0.4 0.19 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}

lock4_side_r (lock_part_4):{ Q:"t(0.4 0.31 0.66) d(0 0 0 1)" , shape:box, size: [0.2 0.03 0.14 0.5], color: [0 0 0], mass: .1,contact:-1}
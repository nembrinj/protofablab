// Parts of the code (nema motor side shaft) taken from
// Parametric NEMA 17 Stepper Motor
// https://www.thingiverse.com/thing:4322777
// Copyright 2020 Jon A. Cruz


// gap for fitting to the pieces
gap = 0.2;

// constants for part fitting on the motor
epsilon = 0.01;
 _shaft_d = 5+gap;
 _shaft_l = 24;
 _notch_l = 15;
 _notch_d = .5;
 _h = 34;
 
 // cubic cupper shaft
 cube_side=10+gap;

// lower section
difference(){
    // lower tube full
    linear_extrude(13,twist=0)
        circle(d=10, $fn=50);

    // lower tube shaft 
    // measurements according to nema 17 motor parameters Jon A. Cruz
    translate([0, 0, -45]){
        difference(){
            // inner cylinder
           translate([0, 0, 2.5])
                cylinder(d = _shaft_d, h = _h - 2.5 + _shaft_l, $fn = 50);
            // notch
           translate([_shaft_d / 2 - _notch_d, -_shaft_d / 2 - epsilon, _h + _shaft_l - _notch_l + epsilon])
                cube([_notch_d + epsilon, _shaft_d + 2 * epsilon, _notch_l + epsilon]);
      }
  }
}

// upper sections
difference(){
    // upper tube full
    translate([0, 0, 13])
        linear_extrude(18,twist=0)
            circle(d=20, $fn=50);
    
    // upper tube shaft
    translate([-cube_side/2, -cube_side/2, 15])
        cube([cube_side,cube_side,cube_side+10]);
}





// stepper wheel

include <constant.scad>
use <utils.scad>

// Main geometrie
motor_piece();

// Core geometric primitives.
module motor_pin() {
    difference() {
        color("Red") cylinder(h=pin_heig,r=pin_diam/2, center=true);
        translate([pin_diam/2,0,0]) cube([pin_diam-pin_flat, pin_diam, pin_heig + 0.2], center=true);
    }
}

module motor_piece() {
    difference() {
        translate([0,0,wheel_height/2+1.0]) wheel(motor_dia, wheel_height, belt_ratio, belt_dia);
        translate([0,0,pin_heig/2-1.0]) motor_pin();
    }
}

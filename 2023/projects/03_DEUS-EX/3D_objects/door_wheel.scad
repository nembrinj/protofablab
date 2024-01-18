// door wheel

include <constant.scad>
use <utils.scad>

// Main geometrie
door_piece();

// Core geometric primitives.
module door_pin() {
    module outcyl() {
        color("Aqua") cylinder(h=door_height, r=door_rad, center=true);
    }
    
    module nob() {
        dis = door_rad - door_size/2;
        intersection() {
            translate([0,dis,0]) outcyl();
            translate([dis,0,0]) outcyl();
            translate([0,-dis,0]) outcyl();
            translate([-dis,,0]) outcyl();
        }
    }
    
    intersection() {
        color("Red") cylinder(h=door_height,r=door_dia/2, center=true);
        nob();
    }
}

module door_piece() {
    whe_diam = 15.0 * 4;
    difference() {
        wheel(whe_diam, wheel_height, belt_ratio, belt_dia);
        door_pin();
    }
}

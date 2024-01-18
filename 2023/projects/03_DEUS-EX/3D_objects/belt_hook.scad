// belt hook

// Global resolution
$fs = 0.1;  // Don't generate smaller facets than 0.1 mm
$fa = 5;    // Don't generate larger angles than 5 degrees

// Main variable
belt_height = 9.00;
hook_length = 25.00;

hole_radius = 1.60;
hole_depth = 3.00;
thickness = 0.80;


// Main geometry
belt_hook();

// Core geometric primitives.

module belt_hook() {
    union() {
        intersection() {
            color("Cyan") translate([0,0,0]) cube([hook_length,belt_height,thickness], center=true);
            cylinder(h=thickness+1,r=hook_length/2,center=true);
        }
        color("Magenta") translate([-hook_length/4,0,hole_depth/2+thickness/2]) cylinder(h=hole_depth,r=hole_radius,center=true);
        color("Magenta") translate([hook_length/4,0,,hole_depth/2+thickness/2]) cylinder(h=hole_depth,r=hole_radius,center=true);
        color("Pink") translate([-hook_length/4,0,,hole_depth/2+thickness/2+hole_depth/2]) cylinder(h=0.5,r=hole_radius+0.6,center=true);
        color("Pink") translate([hook_length/4,0,,hole_depth/2+thickness/2+hole_depth/2]) cylinder(h=0.5,r=hole_radius+0.6,center=true);
    }
}

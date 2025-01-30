use <Arc_Module_2.scad>

$fa=1; // fragment minimum angle 
$fs=1; // fragment minimum size

// MEASURES 

base_thickness = 1;

cup_diameter = 67; // real : 65.6
cup_height = 70;

breadboard_height = 8;
breadboard_width = 54; // real : 53.75

button_diameter = 11.4; // real : 11.35
button_heigth = 4 + base_thickness;

hole_distance = 19;
hole_d = 22;

// CUP MODULES

module CableHole() {
    rotate([0,0,75])
    translate([0,0,breadboard_height-1])
    linear_extrude(height = 5, center = false, twist = 0)
    arc(30, cup_diameter/2 + 2, cup_diameter/2 - 2);
}

module Base() {
    // Button enclosure
    difference() {
        cylinder(h=button_heigth, d=button_diameter + base_thickness, center=false);
        cylinder(h=button_heigth * 3, d=button_diameter, center=true);
    }

    difference() {
    cylinder(h=base_thickness, d=cup_diameter - 0.6);
    union() {
    cylinder(h=base_thickness * 5, d=6, center=true);

    translate([0,-hole_distance,0])
    cylinder(h=base_thickness * 5, d=hole_d, center=true);
        
    translate([0,hole_distance,0])
    cylinder(h=base_thickness * 5, d=hole_d, center=true);

    translate([hole_distance,0,0])
    cylinder(h=base_thickness * 5, d=hole_d, center=true);
        
    translate([-hole_distance,0,0])
    cylinder(h=base_thickness * 5, d=hole_d, center=true);
    }
    }
}

// MODELS

module Cup() {
    difference() {
    difference() {
    difference() {
        // can hold cylinder
        difference() {
            cylinder(h=cup_height, d=cup_diameter + base_thickness, center=false);
            union() {
            cylinder(h=cup_height * 3, d=cup_diameter, center=true);
            }
        }
        // Breadboard enclosure
        cube([breadboard_width, cup_diameter * 2, breadboard_height * 2], center=true);
    }
        translate([0,0,30])
        linear_extrude(height = 70, center = false, twist = 0)
        union(){
            arc(60, cup_diameter/2 + 2, cup_diameter/2 - 2);

            rotate([0,0,120])
            arc(60, cup_diameter/2 + 2, cup_diameter/2 - 2);

            rotate([0,0,-120])
            arc(60, cup_diameter/2 + 2, cup_diameter/2 - 2);
        }
    }
        CableHole();
    }
}
Cup();

// Can holder base
Base();

// Base Stabilizers
module BaseStabilizer(edge_width, number) {
    angle = 10;
    for ( i = [0 : number] ){
        rotate([0, 0, (360/number) * i + angle/2])
        rotate_extrude(angle=angle)
        translate([cup_diameter/2 - edge_width,breadboard_height + 10 - edge_width,0])
        polygon(points=[[edge_width,edge_width],[edge_width,0],[0,edge_width]],convexity=0);
    }
}
BaseStabilizer(3, 6);



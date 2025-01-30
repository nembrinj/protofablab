use <library/Arc_Module_2.scad>

$fa=1; // fragment minimum angle 
$fs=1; // fragment minimum size

// MEASURES 

base_thickness = 1.5;

cup_diameter = 67; // real : 65.6, inside diameter
cup_height = 70;

breadboard_height = 6;
breadboard_width = 54; // real : 53.75

button_diameter = 11.4; // real : 11.35

// CUP MODULES

module CableHole(height=3, angle_on_cup=75, size_angle=15) {
    rotate([0,0,angle_on_cup])
    translate([0,0,breadboard_height-1])
    linear_extrude(height = height+1, center = false, twist = 0)
    arc(size_angle, cup_diameter/2 + 2, cup_diameter/2 - 2);
}

module BreadboardHole() {
    cube([breadboard_width, cup_diameter * 2, breadboard_height * 2], center=true);
}

module CupWhiteSpaces(height=45, nb_spaces=3) {
    for ( i = [0:nb_spaces-1]) {
        translate([0,0,cup_height-height])
        rotate([0,0,(360/nb_spaces)*i])
        linear_extrude(height = cup_height, center = false, twist = 0)
        arc((360/2/nb_spaces), cup_diameter/2 + (base_thickness*2), cup_diameter/2 - base_thickness);
    }    
}

module CupCylinder() {
    difference() {
        cylinder(h=cup_height, d=cup_diameter + (base_thickness*2), center=false);
        cylinder(h=cup_height * 3, d=cup_diameter, center=true);
    }
}

// FLOOR MODULES

module ButtonEnclosure(height=4) {
    functional_height = height + base_thickness;
    difference() {
        cylinder(h=functional_height, d=button_diameter + base_thickness*2, center=false);
        cylinder(h=functional_height * 3, d=button_diameter, center=true);
    }
}

module FloorBase() {
    cylinder(h=base_thickness, d=cup_diameter - 0.6);
}

module FloorWhiteSpaces() {
    // Center hole. Important to be able to remove button.
    cylinder(h=base_thickness * 3, d=button_diameter-2, center=true);
    
    // Hardcoded because calculation becomes complex when configurable.
    for ( i = [0:4]) {
        rotate([0,0,(360/4)*i])
        translate([0,-19,0])
        cylinder(h=base_thickness * 3, d=22, center=true);
    }  
}

// FINAL MODULES

module Cup() {
    difference() {
        CupCylinder();
        union() {
            BreadboardHole();
            CupWhiteSpaces();
            CableHole();
        }
    }
}

module Floor() {
    ButtonEnclosure();

    difference() {
        FloorBase();
        FloorWhiteSpaces();
    }
}

// MAIN

Cup();
Floor();



$fa=1; // fragment minimum angle 
$fs=1; // fragment minimum size

// MEASURES 

base_thickness = 1;

cup_thickness = base_thickness;
cup_diameter = 67; // real : 65.6
cup_height = 20;

breadboard_height = 5;
breadboard_width = 54; // real : 53.75

button_diameter = 11.4; // real : 11.35
button_heigth = 4 + base_thickness;

hole_distance = 19;
hole_d = 22;

// MODELS 

translate([0,0,cup_height])
mirror([0,0,1])
difference() {
    // can hold cylinder
    difference() {
        cylinder(h=cup_height, d=cup_diameter + cup_thickness, center=false);
        cylinder(h=cup_height * 3, d=cup_diameter, center=true);
    }
    // Breadboard enclosure
    cube([breadboard_width, cup_diameter * 2, breadboard_height], center=true);
}

// Can holder base
difference() {
cylinder(h=base_thickness, d=cup_diameter - 1);
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


// Button enclosure
difference() {
    cylinder(h=button_heigth, d=button_diameter + base_thickness, center=false);
    cylinder(h=button_heigth * 3, d=button_diameter, center=true);
}


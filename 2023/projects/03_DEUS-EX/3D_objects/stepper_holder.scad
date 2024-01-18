// motor holder

include <constant.scad>
use <utils.scad>

// Main variable
tot_len = spring_length*0.8+piston_bottom;
tot_dia = spring_outer + 2*piston_thickness;

// Main geometry
motor_holder();

// Core geometric primitives.
module motor_holder() {
    tot_h = motor_height + motor_margin + motor_thickness;
    tot_w = motor_side + motor_margin;
    tot_l = motor_side + motor_margin + 2*motor_thickness;
    fixer_l = motor_side * 2 / 3;
    
    module screw() {
        color("Orange") cylinder(h=motor_thickness+0.1,r=screw_side/2+motor_margin/2,center=true);
    }
    
    module wall(h,w) {
        color("Blue") rotate([90,0,0]) plane(w, h, motor_thickness, 0.15);
    }
    
    module top() {
        difference() {
            color("Cyan") cube([tot_w,tot_l,motor_thickness], center=true);
            
            // center
            color("Red") translate([0,0,0]) cylinder(h=motor_thickness+0.1,r=motor_center/2+motor_margin,center=true);
            
            // screw
            d = motor_screw/2;
            translate([ d, d,0]) screw();
            translate([ d,-d,0]) screw();
            translate([-d, d,0]) screw();
            translate([-d,-d,0]) screw();
        }
    }
    
    module fixer() {
        difference() {
            color("Green") cube([tot_w,fixer_l,motor_thickness], center=true);
            
            // screw
            translate([tot_w/4,-fixer_l/8+motor_thickness/2,0]) screw();
            translate([-tot_w/4,fixer_l/8+motor_thickness/2,0]) screw();
        }
    }
    
    module outer_piston() {
        rotate([0,90,0]) union() {
            color("SteelBlue") translate([0,0,-piston_bottom]) tube(tot_len, tot_dia, spring_outer);
            color("LightSteelBlue") translate([0,0,tot_len/2-piston_bottom/2]) tube(piston_bottom,tot_dia,spring_inner);
        }
    }
    
    union() {
        // sides
        translate([0, tot_l/2-motor_thickness/2,0]) wall(tot_h,tot_w);
        translate([0,-tot_l/2+motor_thickness/2,0]) wall(tot_h,tot_w);
        
        // top
        translate([0,0,tot_h/2-motor_thickness/2]) top();
        
        // sides
        translate([0, tot_w/2+fixer_l/2,-tot_h/2+motor_thickness/2]) fixer();
        translate([0,-tot_w/2-fixer_l/2,-tot_h/2+motor_thickness/2]) rotate([180,0,0]) fixer();
        
        // piston
        translate([(tot_w-tot_len)/2,tot_l/2+fixer_l-motor_thickness,0]) outer_piston();
        translate([0,tot_l/2+fixer_l-motor_thickness,-tot_h/3]) wall(tot_h/3,tot_w);
        translate([tot_w/2-motor_thickness/2,tot_l/2+fixer_l-motor_thickness,-tot_h/4]) rotate([0,0,90]) difference() {
            wall(tot_h/2,tot_dia);
            translate([0,0,tot_h/4]) rotate([90,0,0]) cylinder(h=motor_thickness+0.1,r=spring_inner/2+0.15,center=true);
        }
    }
}

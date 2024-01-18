// piston inside

include <constant.scad>
use <BOSL/metric_screws.scad>

// Main variable
spr_in = spring_inner - 2*slide_diff;
spr_out = spring_outer - 2*slide_diff;

// Main geometry
inner_piston();

// Core geometric primitives.

module inner_piston() {
    rotate([90,0,0]) union() {
        // main tube
        color("Purple") cylinder(h=spring_length*0.8, r=spr_in/2, center=true);
        color("Magenta") translate([0,0,spring_length*0.7]) cylinder(h=spring_length*0.6,r=spr_out/2,center=true);
        translate([0,0,-spring_length*0.8/2-piston_bottom/2]) color("Cyan") cylinder(h=piston_bottom,r=spr_in/2,center=true);
        
        // screw
        translate([0,0,-spring_length*0.8/2-piston_bottom]) intersection() {
            color("Cyan") cylinder(h=piston_bottom*1.2,r=spr_in/2,center=true);
            translate([0,0,-piston_bottom]) rotate([180,0,0]) metric_bolt(size=spr_in-screw_diff, l=piston_bottom, pitch=screw_pitch, details=true);
        }
        
        // fixation point
        translate([0,0,spring_length]) difference() {
            union() {
                color("Red") cube([spring_outer*1.5,piston_thickness*2+bearing_height+1.2,piston_thickness],center=true);
                translate([0,piston_thickness/2+bearing_height/2+0.6,30-piston_thickness/2]) color("Pink") cube([spring_outer*1.5,piston_thickness,60],center=true);
                translate([0,-piston_thickness/2-bearing_height/2-0.6,30-piston_thickness/2]) color("Pink") cube([spring_outer*1.5,piston_thickness,60],center=true);
            }
            color("DeepPink") translate([0,0,40+7+piston_thickness]) rotate([90,0,0]) cylinder(h=2*piston_thickness+bearing_height+1.3,r=bearing_in_dia/2,center=true);
        }
    }
}

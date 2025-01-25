difference()
{

     
translate([0,8,0]){
rotate([90,0,0]){
translate([0,18,0]){
import("midt_esp32_head_block.stl");
}
}
}

translate([-25, -12, -1])
cube([46,25,30]);

}

 translate([-2.5, 16.0, 39.8]) {
 rotate([0,0,135]){
 rotate_extrude(angle=270, convexity = 2, $fn=100)
 translate([7.5, 0, 0])
 square(size=2,center = true);
 }
 
 }
 

translate([0,-24, 29]){
rotate([180,0,0]){
import("midt_esp32_pen_mnt.stl");
}
}

translate([0,8,0]){
rotate([90,0,0]){
translate([0,18,0]){
import("midt_esp32_head_block.stl");
}
}
}
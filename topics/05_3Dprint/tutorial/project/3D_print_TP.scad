//at first : teddy is 50mm when scaling is == 1
//to change the teddy size, please change the value in the scaling call


module head(){
difference(){

//head
translate([0,0,8])
sphere(r = 10, $fn=150);

//limit
translate([-30,-10,-10])
cube([40,20,10]);
}
}

module mouth(){
difference(){
//mouth part
translate([7,0,4])
rotate([90,45,0])
scale([2,2.5,3])sphere(r=2, $fn=100);

//limit
translate([-30,-10,-10])
cube([40,20,10]);
}
}

module ear(){
difference(){
//ear right
translate([0,8,14])
scale([0.75,1,1])sphere(r=4, $fn=100);

//ear right hole
translate([3,8,14])
scale([0.75,1,1])sphere(r=2, $fn=100);
}
}

module body(){
difference(){
//body
translate([-18,0,3])
rotate([0,9,0])
scale([5,3,2.5])sphere(r=3.5, $fn=100);
    
//limit
translate([-40,-10,-10])
cube([50,20,10]);    
}
}

module foot(){
difference(){
//right leg
translate([-28.1,9,4])
rotate([7.5,0,150])
scale([1,2.5,1]) sphere(r=5, $fn=100);
//limit
translate([-50,0,-10])
cube([50,120,10]);
}

difference(){
//right foot
translate([-22.75,17,3.75])
rotate([0,10,9])
scale([1.15,1.35,2]) sphere(r=2.5
, $fn=100); 

//limit
translate([-50,0,-10])
cube([50,80,10]);
    
//paw round right
translate([-20.5,19,3])
rotate([0,7,30])
scale([0.5,1.95,2]) sphere(r=1
, $fn=100);
    
//paw round right
translate([-21.5,20,6])
rotate([0,-7,15])
scale([1,1.65,1.75]) sphere(r=0.5
, $fn=100);

//paw round right
translate([-20.6,18.2,6.85])
rotate([0,-7,10])
scale([1,1.45,1.95]) sphere(r=0.5
, $fn=100);

//paw round right
translate([-20,16.65,5.5])
rotate([7,0,0])
scale([1,1.45,1.95]) sphere(r=0.5
, $fn=100);

}


}

module butt(){
difference(){
//bottom
translate([-26,0,5])
scale([1.25,1.5,1]) sphere(r=8, $fn=100);
//limit
translate([-50,-10,-10])
cube([50,20,10]);
    
}
//pompom on the bottom
translate([-33,0,9.75])
 sphere(r=2, $fn=100);
}

module nose(){
//nose
translate([10,0,5])
rotate([0,90,0])

minkowski(){
cylinder(1.25,1.25,1.25,$fn=3);
sphere(r=0.5, $fn=100);
}
}

module eye(){
difference(){
//eye
translate([9,-2.75,10])
rotate([0,-9,-16])
scale([0.5,1.5,2]) sphere(r=1.5
, $fn=100);

//inner eye
translate([10,-2.75,8.5])
rotate([0,-9,-16])
scale([0.75,1.5,2]) sphere(r=0.75
, $fn=100);
}
}

module arm(){

difference(){
//right arm
translate([-10,10,2])
rotate([0,0,-70])
scale([1,2.5,1]) sphere(r=4.5, $fn=100);
//limit
translate([-20,0,-10])
cube([30,30,10]);

}

difference(){
//right hand
translate([-2,12,2])
rotate([0,90,0])
scale([1.25,1.5,2]) sphere(r=2.5
, $fn=100);
    
//limit
translate([-20,0,-10])
cube([30,30,10]);
}
}
module teddy(){
union(){
ear();
mirror([0,1,0]) ear();
mouth();
body();
foot();
mirror([0,1,0]) foot();
butt();
nose();
mirror([0,1,0]) eye();
eye();
mirror([0,1,0]) arm();
arm();
head();
}
}
module scaling(size){
union(){
translate([15*size,0,0])
scale([size,size,size]) teddy();
}
}
scaling(0.25);
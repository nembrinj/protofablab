/* Explanation of arc() module (Version_02):
    This module is for creating 2D arcs.

IMPORTANT NOTE: If you've been using Version_01 be aware that some of the paramotor names and their sequence have changed.

	arc(angle or a			= angle of arc, 
        r1 or r or d1 or d  = radius or diameter 1, 
        r2 or d2            = radius or diameter 2 (optional),
		turn				= turns/rotates the arc
        ffn 				= Fixed_Fragments_Number irrespective of angle 
        rfn 				= Round_Fragments_Number to an integer
		info				= echo a reminder of parameters

Parameters:
angle or a:
    Angle of the arc. Value range = [-360 : 360]
    If undefined the default value = 360 
 
r1 or r: 
    Sets the radius of the arc. Alternatively by specifying the variable name "d" or "d1" you can set diameter instead of the radius. 
    If undefined the default radius value = 1 


r2: 
    Gives the option of a second radius to turn the circle section into a ring section. Again by specifying the variable name "d2" you can set diameter instead of the radius
    If undefined the default value = 0  

turn or t:
	Turns/rotates the arc on the Z axis. Inputs can be positive or negative values.      

ffn: Fixed_Fragments_Number (default=OFF)
    Gives the option of setting a fixed number of fragments in the arc, irrespective of the angle size. 
    If ffn = undef it is off, and the fragment number will be calculated by rfn (Round_Fragments_Number) using $fa or $fn, this is the default. 
    

    By default, the number of fragments in the arc is set by $fn or $fa, this value will determine the number of fragments in a full circle. The number of fragments in the arc is proportional to the size of its angle, so, an arc of 180° will have half as many fragments as an arc of 360°. This keeps the fragments as close to a constant size as possible.

    If $fn=undef or $fn=0, which it is by default, the number of figments is set by $fa (minimum angle), which has a default value of 12°.  (360°/12° = 30 figments in a circle)

rfn: Round_Fragments_Number
    The number of fragments has to be an integer, however dividing the angle by $fn will more often than not return a floating-point number which has to be rounded up or down too the nearest whole number.
    If rfn = undef or "down" or 0 it will round down. Rounding down is the default, this is because this will match segments with rotate_extrude.
    If fragRound = "near" or 1 it will round to the nearest integer.
    If fragRound = "up" or 2 it will round up.
	If fragFixed is given a value it will overide fragRound parameter.
   
info: (default=OFF)
	If info = true echo a list of the input parameters to the consol as a reminder of what they are and their order.
*/


//>>>>>>>>>>>>> Call arc module <<<<<<<<<<<<< 

arc(a=270, r1=10, r2=5, t=0, ffn=undef, rfn="near", info=true);


//>>>>>>>>>>>> Declair arc module <<<<<<<<<<<< 
module arc(angle, r1, r2, turn, ffn, rfn, a, r, d1, d2, d, t, info=false){
//warnings
if(		//if r1_ is defined in more than one way
	(is_num(r)  	? 1:0) + 
	(is_num(r1) 	? 1:0) +
	(is_num(d)  	? 1:0) +
	(is_num(d1) 	? 1:0) >1
	||	//if r2_ is defined in more than one way
	(is_num(r2) 	? 1:0) + 
	(is_num(d2) 	? 1:0) >1
	||	//if angle is defined in more than one way
	(is_num(angle) 	? 1:0) +
	(is_num(a)	   	? 1:0) >1
	||	//if turn is defined in more than one way
	(is_num(turn) 	? 1:0) +
	(is_num(t)	   	? 1:0) >1
) 
	echo("WARNING: There is more than one variable describing the same dimention of the arc. Remove redundent variables"); 

a_ = is_num(angle) ? angle :
	 is_num(a)		? a : 360;

//radius 1
// r, r1, d and d1 are all valid inputs to describe the radius. 
// if radius is not input in any valid form it will default to a value of 1 
r1_ = !is_num(r1) && !is_num(r) && !is_num(d1) && !is_num(d)  ? 1  : 
//if d1 is a valid input	
	is_num(d1) ? d1/2  :
//if d is a valid input, if not, r1=0 (if r2 or d2 are valid inputs)
	is_num(d) ? d/2  : 
//if r1 is a valid input	
	is_num(r1) ? r1  :
//if r is a valid input	
	is_num(r) ? r  : 0;

//radius 2
//if d2 is a valid input, if not r2=0 (default)
r2_ = is_num(d2) ? d2/2 :
//if r2 is a valid input
	 is_num(r2) ? r2 : 0;


fragFixed_=	is_num(ffn) ? ffn : undef;

fragRound_ = rfn==undef ? 0 : rfn;

turn_ =	is_num(turn)  	? turn : 
		is_num(t)		? t : 0;

//Prevent the arc from intersecting itsef and make negative angle values positive.
angCap  =  (a_ == undef || a_ >   360) ? 360  : 
           (a_ <  0     && a_ >= -360) ? a_*-1 : 
                           a_ <  -360  ? 360  : a_; 
             
//set number of fragments 
// if number is below a threshold for the angle, apply minimum number of fragments 
frag = min_frag( 
fragFixed_==undef ? ($fn==undef || $fn==0 ? ((360/($fa==undef ? 90:$fa))/360)*angCap : (($fn/360)*angCap)) : fragFixed_);

//Round the number of fragments ether up or down too an intager. min num frag=1
fragNum = fragRound_==undef || fragRound_=="down"|| fragRound_==0 ? (floor(frag)<1?1:floor(frag))://round down (defult)
                              fragRound_=="near" || fragRound_==1 ? (round(frag)<1?1:round(frag))://round  
                              fragRound_=="up"   || fragRound_==2 ?  ceil (frag)://round up 
                                                                  (floor(frag)<1?1:floor(frag));//round down


//number of vectors
l=len(points(r1_));

color("gold")
	rotate([0,0,turn_])
		mirror([0, (a_==undef || a_>0) ? 1 : 0])//flip arc if angle < 0 
			rotate([0,0,-90])
if(((r1_==undef || r1_<=0) && (r2_==undef || r2_<=0)) || r1_==r2_ || angCap==undef || angCap==0 ){
    //The shape has no dimentions so do nothing.
}
else{
    if( r2_==undef || r2_<=0){
    //Outer radeus defined by r1_, No inner radius.
    polygon(concat([for (i=[0:l-1]) points(r1_)[i]], [[0,0]]));
}
else{
    if( r1_==undef || r1_<=0){
    //Outer radeus defined by r2_, No inner radius.
    polygon(concat([for (i=[0:l-1]) points(r2_)[i]], [[0,0]]));
}
else{
    //Inner and outer radiuses
    polygon([for (i=[0:(l*2)-1]) i<l ? points(r1_)[i] : points(r2_)[(l*2)-(i+1)]]);
}
}
}


//FUNCTIONS
//calculate the postion of the points.
function points (r) = [ for (a = [0 : angCap/round(fragNum) : angCap]) [ r * sin(a), r * cos(a) ] ];

//set a minimum number of Number_of_fragments to a given angle size.
function min_frag(fragIn)=
(fragIn==undef || fragIn<3) && angCap>=270 ? 3 :
(fragIn==undef || fragIn<2) && angCap>=180 ? 2 : 
(fragIn==undef || fragIn<1) && angCap>= 90 ? 1 : fragIn;
//(fragIn==undef || fragIn<1) && angCap>=  0 ? 1 : fragIn;

//Info
//will echo a reminder of 
if(info){
	echo("Paramerters of arc() module: ");	
	echo("1:  a = angle of arc, [-360 : 360] ");
	echo("2:  r1 = radius 1, [number]");
	echo("3:  r2 = radius 2, [number]");
	echo("4:  t = turns/rotates the arc [number]");
	echo("5:  ffn = Fixed_Fragments_Number [number]");
	echo("6:  rfn = Round_Fragments_Number [down, near, up] ");
	echo("7:  info = echo a reminder of parameters [false, true] ");
}
}

/* Publication Notes 

arc() Module for OpenSCAD (Version 02) by SavageRodent
https://www.thingiverse.com/thing:5186085
Last updated: 11/06/2024
Published under: the Creative Commons CC - BY 3.0 licence
Licence details here: https://creativecommons.org/licenses/by/3.0/
*/
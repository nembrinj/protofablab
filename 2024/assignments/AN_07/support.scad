// ESP32 -VEML7700 support parametric model
//   
//    syntax: 
//        //Description
//        variable=value; //Parameter
//        
//        This type of comment tells the name of group to which parameters below
//        this comment will belong 
//    
//       /*[ group name ]*/ 
//
//Below comment tells the group to which a variable will belong

/*[Properties of the ESP support]*/

//The depth of the ESP32 Feather support
ESP_depth = 200;//[150 : 800]


/*[Properties of the VEML support]*/

//The angle of the VEML support
VEML_angle = 45;//[0 : 90]

//The altitude of the VEML support
VEML_altitude = 30;//[10:55]

//The X shift of the VEML support
link_shift_X = 10;//[0 :20]

//The Y shift of the VEML support
link_shift_Y = -5;//[-20 :20]


/*[ Content To be written ] */

//Text to be engraved 
Message = "H3"; //["AB","CD...", "pqr", "XYZ"]

//Text scale
text_scale = 0.4;


/*[ Rendering settings ] */

//Facet angle
$fa = 4;
//Facet size
$fs = 2; 


/* [Hidden] */ // non-parametric values

//The width of the ESP32 Feather support
ESP_width = 100;//[50 : 500]
//The height of the ESP Feather support
ESP_height = 80;//[10 : 200]
//The thickness of the ESP32 Feather 
ESP_thickness = 70;//[50 : 200]
//The vertical shift of the ESP32 Feather 
ESP_shift = 10;//[10 : 100]

// ESP support pin comb lower height
ESP_comb_height1=40;   
// ESP support pin comb higher height
ESP_comb_height2=70;   
// ESP support pin comb width
ESP_comb_width=ESP_width-ESP_thickness+10;
// ESP support pin comb depth
ESP_comb_depth=10; 

//The depth of the VEML support
VEML_depth = 10;//[50 : 200]
//The width of the VEML support
VEML_width = 200;//[100 : 500]
//The height of the VEML support
VEML_height = 100;//[10 : 200]
//The vertical shift of the VEML pin through 
VEML_shift = 20;//[10 : 100]
// VEML pillar thickness
VEML_thickness = 3;//[2:5]


// The space between the pins (norm = 2.54)
pin_space = 2.54;

// ESP support
module esp_support() {
scale([0.1, 0.1, 0.1])
    translate([0,0,ESP_height/2])
    difference() {
        // base U shape
        cube([ESP_width,ESP_depth,ESP_height], center = true);
        translate([0, 0,ESP_shift])
            cube([ESP_thickness,ESP_depth+10,ESP_height], center=true);
        // cubes to cut comb dents
        translate([ESP_thickness/2,0,0])
            for (dy=[-ESP_depth*10:pin_space*100:ESP_depth*10]) {
                translate([0,dy/10,ESP_comb_height1/2])
                    cube([ESP_comb_width,ESP_comb_depth,ESP_height],center=true);
            };
        // cube to cut at comb height    
        translate([ESP_width/2,0,ESP_comb_height2/2])
            cube([ESP_width,ESP_depth+10,ESP_height],center=true);
    }
}

// VEML support
module veml_support(thick,height) {
    translate([0,0,VEML_altitude]) {
        // uncomment to get rid of supports //translate([0,VEML_depth/10*cos(VEML_angle),VEML_depth/10*cos(VEML_angle)/tan(VEML_angle)])
        rotate([-VEML_angle,0,0])
        //VEML_height/2*cos(VEML_angle)
        scale([0.1, 0.1, 0.1]) 
        translate([0,-VEML_depth/2,VEML_height/2])
        difference() {
            // support plate
            cube([VEML_width,VEML_depth,VEML_height], center = true);
            // opening in support
            translate([0, 0,VEML_shift])
                cube([5*pin_space*10.0,VEML_width+10,10], center=true);
        } 
        
        // pillar with precise angle cut
        translate([0,thick/2,(-height+thick/tan(VEML_angle))/2])
        difference() {
            cube([VEML_width/10,thick,height+thick/tan(VEML_angle)], center=true);
            translate([0,0,(height+thick/tan(VEML_angle))/2])
                rotate([-VEML_angle,0,0])
                cube([100,thick*cos(VEML_angle),100],center=true);
        }
    }
}

// link between ESP and VEML support
module link(shiftX) {
translate([0,-VEML_thickness,0])
    // simple extrusion of a triangle that extends into the support pillar
    linear_extrude(height=2)
    polygon([[0,0],[shiftX+10,0],[0,10]]);
}

// text
module text_engraved() {
translate([0,0,-2.5])
    scale([text_scale,text_scale,text_scale])
    rotate([0,0,90])
    linear_extrude(height = ESP_shift+10) {
        translate([0, -4]) text(Message, halign = "center");
    }
}

// whole model
difference() {
    union() {
        esp_support();
        translate([ESP_width/20,link_shift_Y,0]) {
            translate([VEML_width/20+link_shift_X,-VEML_thickness,0])
            veml_support(VEML_thickness,VEML_altitude);
            link(link_shift_X);
        }
    };
    text_engraved();
}

// utils

// Core geometric primitives.
tube(50, 20,10);
translate([0,100,0]) plane(150, 100, 5, 0.1);

module tube(length, out_dia, in_dia) {
    difference() {
        cylinder(h=length,r=out_dia/2,center=true);
        cylinder(h=length+0.1,r=in_dia/2,center=true);
    }
}

module wheel(whe_diam, whe_heig, pin_ratio, pin_dia) {
    border_z = whe_heig+1.0;
    border_d = whe_diam+5.0;
    nb_pins = pin_ratio * whe_diam;
    alpha = 360 / nb_pins;
    
    module border() {
        color("Yellow") cylinder(h=1.0, r=border_d/2, center=true);
    }
    
    module gear() {
        module pins() {
            color("Purple") for (i = [1:nb_pins]) {
                translate([sin(i*alpha)*whe_diam/2,cos(i*alpha)*whe_diam/2,0]) cylinder(h=whe_heig+0.1, r=pin_dia/2, center=true);
            }
        }
        difference() {
            color("Green") cylinder(h=whe_heig, r=whe_diam/2, center=true);
            pins();
        }
    }
    
    union() {
        translate([0,0, border_z/2]) border();
        gear();
        translate([0,0,-border_z/2]) border();
    }
}

module plane(width, length, height, fill) {
    
    module grid(size,cell_hole,cell_wall) {
        dx=cell_hole*sqrt(3)+cell_wall*sqrt(3);
        dy=cell_hole+cell_wall;
        
        translate(-size/2) for (i = [0:ceil(size[0]/dx)*2]) {
            for (j = [0:ceil(size[1]/dy)]) {
                translate([i*dx/2,j*dy - i%2*dy/2,-0.1]) cylinder(d=cell_hole,h=size[2]+0.2,$fn=6);
                if (i%2 == 1) {
                    translate([i*dx/2,j*dy + i%2*dy/2,-0.1]) cylinder(d=cell_hole,h=size[2]+0.2,$fn=6);
                }
            }
        }
    }
    
    module create_grid(size,cell_hole,wall) {
        b = 2*wall;
        b_size = [size[0]-2*b,size[1]-2*b,size[2]+0.2];
        
        union() {
            difference() {
                cube(size=size, center=true);
                cube(size=b_size, center=true);
            }
            difference() {
                cube(size=size, center=true);
                grid(size=b_size, cell_hole=cell_hole, cell_wall=wall);
            }
        }
    }
    
    size = [width, length, height];
    wall = fill * min(width, length) / 2;
    hole = wall /fill * (1-fill);
    test = (min(width, length) - 2* wall) * fill / wall*2;
    test2 = ceil(test);
    hole2 = (min(width, length) - 2* wall) / test2 - wall/2;
    create_grid(size=size,cell_hole=hole2,wall=wall);
}

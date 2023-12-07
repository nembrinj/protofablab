    include <constructive-compiled.scad>
    // units are in millimeter
    $fn=21; // directs the number of faces for round pieces
    
    dx = 34;
    dy = 83;
    dz = 40;
    
    diam = 4.5;
    
    extra = 10;
    thickness = 1.5;
    
    g(TOUP() ) 
    {
    // the base 
    box(x=dx+extra,y=dy+extra,z=3);
    box(x=dx+extra+2*thickness,y=dy+extra+2*thickness,z=thickness);
     
    // the supports
    two()
      g( reflectX(sides()), X(dx/2), Y(-dy/2) ,TOUP(), height(10), solid(true) )
      {
        tube(d=diam);
        box(x=6,y=6,z=7)
        g( Y(dy)) tube(d=diam) box(x=6,y=6,z=7)
        ;
      }  
    }
    
    // the box
    TOUP()
    clear(blue)
    {
    
      assemble() {
          g(Z(thickness)) 
          {      
             add()   
             box(x=dx+extra+2*thickness,y=dy+extra+2*thickness,z=dz-thickness);
            
             remove()Z(-2*thickness) 
             box(x=dx+extra,y=dy+extra,z=dz);  
          }      
      }
    } 
    

    
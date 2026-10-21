from pythonscad import *

eps=0.1
#M4
#head_thickness=3 + eps
#head_diameter=7.66 + eps
#diameter=4 + eps

#M2.5
head_thickness=1.4 + eps
head_diameter=4.9/0.866 + eps
diameter=2.5 + eps

# Mx hole
hole_depth=10
hole = (cylinder(d=diameter,h=10,fn=20) - [0,0,hole_depth]) \
+ circle(r=head_diameter/2,fn=6).linear_extrude(height=head_thickness)
#hole.show()


distance_between_holes=12 

base_depth=16
drawing = osimport("outline.svg", dpi=300)
base = drawing.rotx(90).linear_extrude(height=base_depth)

base = base.fillet(.3,fn=5)

# sensor hole
sensor_height=12
sensor_length=12
sensor_thick=1
sbox = cube([10,sensor_length,sensor_thick],center=True)
base -= sbox + [40,-base_depth/2,sensor_height+sensor_thick/2]
#sbox.show()

# M4 holes
base -= hole ^ [[4,4+distance_between_holes],-base_depth/2,2.1]

# general fillet
final = base#.fillet(.2,fn=5)

#mask.show()

export(final,"test.stl")
final.render().show()


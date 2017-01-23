
#Generate a circle trajectory

#Center of circle
z_center = 0.7
y_center = 0.0


#Radius of circle
radius = 0.2

#Angle Step width
angle = 0.0 : 0.01 : 2*pi;

points = [];
point_vels = [];

for i = 1: size(angle,2)
  points_x = 0.4;
  points_y = y_center + radius * cos(angle(i));
  points_z = z_center + radius * sin(angle(i));
  points = [points ; points_x points_y points_z 1000.0 1000.0 1000.0 1000.0];
  
  point_vels = [point_vels ; 0.0 0.0 0.0 0.0 0.0 0.0];
end

points
point_vels
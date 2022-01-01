clear all ; close all ; clc ; 

% Init data
Data = csvread("dist-angle.csv");
timeD = Data(:,1); initTime = timeD(1); 
dist = Data(:,2);
anglex = Data(:,3);
angley = Data(:,4);

duration = 73;
samples = size(timeD);
delay_bwt = (duration/samples(1))/10;
node_x = 0;
node_y = 0;
node_z = 0;

obj_x= node_x + (dist.*sind(anglex));
obj_y= node_y + (dist.*sind(angley));
obj_z= node_z + dist;

figure();
x_l = 1;
y_l = 1;
z_l = 1;

for i = 1:size(timeD)
   xlim([-2 1.5]);
   ylim([-1 15]);
   zlim([-2 13]);
   view(28,-4);
   cla;
   stem3(node_x, node_y, node_z, 'LineStyle','none','Marker', 's', 'Color', 'r', 'MarkerSize', 32, 'LineWidth', 4);
   hold on;
   stem3(obj_x(i), obj_y(i), obj_z(i), 'LineStyle','none', 'Marker', 'o', 'Color', 'b', 'MarkerSize', 20, 'LineWidth', 2);
   hold on;
   plot3([node_x obj_x(i)], [node_y obj_y(i)], [node_z obj_z(i)], 'Color', 'b');
   grid off;
   xlabel('X') ; ylabel('Y'); zlabel('Z')
   %disp(i);
   pause(delay_bwt);
end

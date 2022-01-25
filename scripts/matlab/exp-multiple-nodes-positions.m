clear all ; close all ; clc

markSize = 20;
lineWidth = 2;

X = [-1.5 1.5 ; -1.5 1.5];
Y = [-1.5 -1.5 ; 1.5 1.5];
Z = [0 0 ; 0 0];

P_X = [-1 -1 1 1];
P_Y = [-1 1 -1 1];
P_Z = [1.36 1.36 1.36 1.36];

r=0.5;
teta=-pi:0.01:pi;
x1=r*cos(teta);
y1=r*sin(teta);

r2=1;
teta=-pi:0.01:pi;
x2=r2*cos(teta);
y2=r2*sin(teta);

dec = 40;

cpp_est = [0, 0, 0 
0.059354, 0.0173186, -0.0891379
-0.978573, -0.0708931, -0.0689981
-0.0874214, 1.07524, -0.0560035
1.04509, 0.0492266, -0.0740765
0.164169, -1.00916, -0.00814798
0.120037, -0.487722, -0.0617424
-0.226777, -0.425303, -0.0731021
-0.445583, -0.0545402, -0.086871
-0.266883, 0.30435, -0.0255646
-0.0151243, 0.484998, -0.057846
0.345903, 0.345903, -0.0674879
0.565318, 0.0651951, -0.0825759
0.416937, -0.344639, -0.0828228];






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ball_radious = 0.144;
x=1;y=2;z=3;

% Distances from each node
% Close to real
%n_dist = [1.820, 1.539, 2.488, 2.471, 1.510, 1.593, 1.473, 1.627, 1.914, 2.129, 2.209, 2.138, 1.837                    
%          1.847, 1.560, 1.507, 2.519, 2.522, 2.146, 1.927, 1.643, 1.478, 1.610, 1.919, 2.168, 2.225
%          1.765, 2.475, 1.514, 1.469, 2.396, 2.040, 2.157, 2.071, 1.831, 1.568, 1.427, 1.547, 1.836
%          1.839, 2.510, 2.499, 1.515, 1.532, 1.617, 1.870, 2.139, 2.223, 2.155, 1.877, 1.591, 1.488];

% Estimated
n_dist = [1.886, 1.248, 2.273, 2.333, 1.248, 1.641, 1.502, 1.611, 1.847, 2.061, 2.162, 2.162, 1.847                    
          1.927, 1.343, 1.198, 2.333, 2.396, 2.162, 1.970, 1.672, 1.528, 1.611, 1.886, 2.110, 2.273
          1.809, 2.333, 1.231, 1.182, 2.162, 2.014, 2.162, 2.110, 1.809, 1.583, 1.453, 1.528, 1.773
          1.886, 2.273, 2.396, 1.323, 1.182, 1.555, 1.773, 2.061, 2.110, 2.110, 1.886, 1.611, 1.528];
n_dist = n_dist + ball_radious;

% Each node's position
n = [-1.0000001 -0.9999999 1.3580001
     -0.9999999 1.00000002 1.3580000
      0.9999997 1.00000001 1.3579999
      0.9999999 -0.9999998 1.3580001];

% Test case
te=1;
ALL = [0 0 0];
A = [ 1,-2*n(1,x),-2*n(1,y),-2*n(1,z)
      1,-2*n(2,x),-2*n(2,y),-2*n(2,z)
      1,-2*n(3,x),-2*n(3,y),-2*n(3,z)
      1,-2*n(4,x),-2*n(4,y),-2*n(4,z)];

for t = 1:13
disp([num2str(n_dist(1,t)), '    ', num2str(n_dist(2,t)), '    ', num2str(n_dist(3,t)), '    ',num2str(n_dist(4,t))]);
B = [n_dist(1,t)^2 - n(1,x)^2 - n(1,y)^2 - n(1,z)^2
    n_dist(2,t)^2 - n(2,x)^2 - n(2,y)^2 - n(2,z)^2
    n_dist(3,t)^2 - n(3,x)^2 - n(3,y)^2 - n(3,z)^2
    n_dist(4,t)^2 - n(4,x)^2 - n(4,y)^2 - n(4,z)^2];
 
T = A\B;
disp([num2str(t),')          ', num2str(T(1)), '           ', num2str(T(2)), '           ', num2str(T(3)), '           ', num2str(T(4))]);
disp([num2str(t),')          O:', num2str(T(1)), '           E:',num2str(T(2)^2 + T(3)^2 + T(4)^2) ]);
disp(' ');
R = [T(2) T(3) T(4)];
ALL = [ALL ; R];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
%surf(X, Y, Z,'FaceColor','white','edgecolor','none');
xlim([X(1,1) X(1,2)]);
ylim([Y(1,1) Y(2,1)]);
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on;
stem3(P_X(1), P_Y(1), P_Z(1), 'MarkerSize', markSize, 'Color', 'blue','LineWidth',lineWidth, 'MarkerEdgeColor', 'red', 'Marker', 's');
hold on;
stem3(P_X(2), P_Y(2), P_Z(2), 'MarkerSize', markSize, 'Color', 'blue','LineWidth',lineWidth, 'MarkerEdgeColor', 'red', 'Marker', 's');
hold on;
stem3(P_X(3), P_Y(3), P_Z(3), 'MarkerSize', markSize, 'Color', 'blue','LineWidth',lineWidth, 'MarkerEdgeColor', 'red', 'Marker', 's');
hold on;
stem3(P_X(4), P_Y(4), P_Z(4), 'MarkerSize', markSize, 'Color', 'blue','LineWidth',lineWidth, 'MarkerEdgeColor', 'red', 'Marker', 's');
hold on; 
plot3(x1,y1,zeros(1,numel(x1)),'MarkerSize',30,'Color', 'black')
hold on; 
plot3(x2,y2,zeros(1,numel(x2)),'MarkerSize',30,'Color', 'black' )
hold on; 
stem([0 0 0 0 0],[-1 -0.5 0 0.5 1],'MarkerSize',30,'Color', 'black','LineStyle', 'none' ,'LineWidth',lineWidth )
hold on; 
stem([-1 -0.5 0 0.5 1],[0 0 0 0 0],'MarkerSize',30,'Color', 'black','LineStyle', 'none' ,'LineWidth',lineWidth )
hold on; 
stem3([r*sin(dec) -r*sin(dec) r*sin(dec) -r*sin(dec)],[r*cos(dec) r*cos(dec) -r*cos(dec) -r*cos(dec)], [0 0 0 0],'MarkerSize',30,'Color', 'black', 'LineStyle', 'none' ,'LineWidth',lineWidth)
hold on;
plot3([0 0], [-1.5 1.5], [0 0],'Color', 'k');
for t = 2:14
    height = 0;%ALL(t,3);%ALL(t,3);
%hold on ; stem3(ALL(t,1), ALL(t,2), height,'MarkerSize',20,'Color', 'green','LineStyle', 'none' ,'LineWidth',5);
%text(ALL(t,1), ALL(t,2), height+0.1, int2str(t-1), 'FontSize', 20, 'Color', 'b')

hold on; stem3(cpp_est(t,1), cpp_est(t,2), cpp_est(t,3),'MarkerSize',20,'Color', 'green','LineStyle', 'none' ,'LineWidth',5);
text(cpp_est(t,1), cpp_est(t,2), cpp_est(t,3) + 0.1, int2str(t-1), 'FontSize', 20, 'Color', 'b')
end
colors_t = ['b' 'r' 'g' 'c'];
for t = 1:0
 location = 10;
 Rg = n_dist(t,location);
 [SX SY SZ] = sphere(100); 
 SX = SX * Rg + n(t,x); 
 SY = SY * Rg + n(t,y); 
 SZ = SZ * Rg + n(t,z); 
 mesh(SX,SY,SZ,'Facecolor','none','EdgeColor',colors_t(mod(t,4)+1)); hold on
end

text(0, 0, 0.1, '1', 'FontSize', 20, 'Color', 'r')
text(-1, 0, 0.1, '2', 'FontSize', 20, 'Color', 'r')
text(0, 1, 0.1, '3', 'FontSize', 20, 'Color', 'r')
text(1, 0, 0.1, '4', 'FontSize', 20, 'Color', 'r')
text(0, -1, 0.1, '5', 'FontSize', 20, 'Color', 'r')
text(0, -0.5, 0.1, '6', 'FontSize', 20, 'Color', 'r')
text(-r*sin(dec), r*cos(dec), 0.1, '7', 'FontSize', 20, 'Color', 'r')
text(-0.5, 0, 0.1, '8', 'FontSize', 20, 'Color', 'r')
text(-r*sin(dec), -r*cos(dec), 0.1, '9', 'FontSize', 20, 'Color', 'r')
text(0, 0.5, 0.1, '10', 'FontSize', 20, 'Color', 'r')
text(r*sin(dec),  -r*cos(dec), 0.1, '11', 'FontSize', 20, 'Color', 'r')
text(0.5, 0, 0.1, '12', 'FontSize', 20, 'Color', 'r')
text(r*sin(dec), r*cos(dec), 0.1, '13', 'FontSize', 20, 'Color', 'r')

text(-1, -1, 0.1, 'Node 1', 'FontSize', 12, 'Color', 'r')
text(-1, 1, 0.1, 'Node 2', 'FontSize', 12, 'Color', 'r')
text(1, 1, 0.1, 'Node 3', 'FontSize', 12, 'Color', 'r')
text(1, -1, 0.1, 'Node 4', 'FontSize', 12, 'Color', 'r')





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
x=r*cos(teta);
y=r*sin(teta);

r2=1;
teta=-pi:0.01:pi;
x2=r2*cos(teta);
y2=r2*sin(teta);

dec = 40;

figure(1)
surf(X, Y, Z,'FaceColor','white','edgecolor','none'); 
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
plot3(x,y,zeros(1,numel(x)),'MarkerSize',30,'Color', 'black')
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



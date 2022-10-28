clear all; close all; clc

axisFontSize = 30;
fontSize = 30;

ball_size_m = 0.144;

focal_X_mm = 9.113812935295416e+02;
focal_Y_mm = 9.113086377881884e+02;

img_W_pix = 1280;
img_H_pix = 720;

sensor_x_mm = 1354.724121;
sensor_y_mm = 1354.724121;

fov_X_deg = 2 * atan(img_W_pix  / (2 * focal_X_mm)) * 180.0 / pi ;
fov_Y_deg = 2 * atan(img_H_pix  / (2 * focal_Y_mm)) * 180.0 / pi ;


pixel_x = [10 : 1 : 250];
pixel_y = [10 : 1 : 250];

oper_x = (focal_X_mm * ball_size_m * img_W_pix)/(sensor_x_mm)
oper_y = (focal_Y_mm * ball_size_m * img_H_pix)/(sensor_y_mm)

dist_x = oper_x ./ pixel_x;
dist_y = oper_y ./ pixel_y;

figure();
p = plot(pixel_x, dist_x);
set(gca,'FontSize',axisFontSize);
set(p, 'linewidth', 2);
xlabel("Object's size (pixels)"); ylabel('Distance (m)');
hold on;
p = plot([9 9],[0 14]);
set(gca,'FontSize',axisFontSize);
set(p, 'linewidth', 2);
text(8, -0.3, '10', 'FontSize', 24);
title('Pixels - Distance relationship for range estimation', 'FontSize', fontSize+4);

hold on;
pix     = 100
pixId   = pix - 10 + 1;
dist    = dist_x(pixId)
p       = plot(pix, dist, 'or'); 
set(gca,'FontSize',axisFontSize);
hold on; plot([pix pix],[0 14]); hold on; plot([0 250],[dist dist]);

hold on;
pix2    = pix + 1
pixId2  = pix2 - 10 + 1;
dist2   = dist_x(pixId2)
p       = plot(pix2, dist2, 'or'); 
set(gca,'FontSize',axisFontSize);
hold on; plot([pix2 pix2],[0 14]); hold on; plot([0 250],[dist2 dist2]);



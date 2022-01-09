clear all; close all; clc;

n = [1:1:13];

n1Data = csvread("n1.csv");
n1_dist = n1Data(:,5); 
n1_act_distances = [1.820 ; 1.539 ; 2.488 ; 2.471 ; 1.510 ; 1.593 ; 1.473 ; 1.627 ; 1.914 ; 2.129 ; 2.209 ; 2.138 ; 1.837];
n1_error = sum(abs(n1_dist - n1_act_distances));
n1_error_in = sum(abs(n1_dist(5:end) - n1_act_distances(5:end))) + abs(n1_dist(1) - n1_act_distances(1));
n1_error_out = sum(abs(n1_dist(2:5) - n1_act_distances(2:5)));

n2Data = csvread("n2.csv");
n2_dist = n2Data(:,5); 
n2_act_distances = [1.847 ; 1.560 ; 1.507 ; 2.519 ; 2.522 ; 2.146 ; 1.927 ; 1.643 ; 1.478 ; 1.610 ; 1.919 ; 2.168 ; 2.225];
n2_error = sum(abs(n2_dist - n2_act_distances));
n2_error_in = sum(abs(n2_dist(5:end) - n2_act_distances(5:end))) + abs(n2_dist(1) - n2_act_distances(1));
n2_error_out = sum(abs(n2_dist(2:5) - n2_act_distances(2:5)));

n3Data = csvread("n3.csv");
n3_dist = n3Data(:,5); 
n3_act_distances = [1.765 ; 2.475 ; 1.514 ; 1.469 ; 2.396 ; 2.040 ; 2.157 ; 2.071 ; 1.831 ; 1.568 ; 1.427 ; 1.547 ; 1.836];
n3_error = sum(abs(n3_dist - n3_act_distances));
n3_error_in = sum(abs(n3_dist(5:end) - n3_act_distances(5:end))) + abs(n3_dist(1) - n3_act_distances(1));
n3_error_out = sum(abs(n3_dist(2:5) - n3_act_distances(2:5)));

n4Data = csvread("n4.csv");
n4_dist = n4Data(:,5); 
n4_act_distances = [1.839 ; 2.510 ; 2.499 ; 1.515 ; 1.532 ; 1.617 ; 1.870 ; 2.138 ; 2.223 ; 2.155 ; 1.877 ; 1.597 ; 1.488];
n4_error = sum(abs(n4_dist - n4_act_distances));
n4_error_in = sum(abs(n4_dist(5:end) - n4_act_distances(5:end))) + abs(n4_dist(1) - n4_act_distances(1));
n4_error_out = sum(abs(n4_dist(2:5) - n4_act_distances(2:5)));

error_out = (n1_error_out + n2_error_out + n3_error_out + n4_error_out)/(4+ 4 +4 +4)
error_gen = (n1_error + n2_error + n3_error + n4_error) / (13 + 13 + 13 + 13)
error_in = (n1_error_in + n2_error_in + n3_error_in + n4_error_in)/(9 + 9 + 9 + 9)

figure()
set(gcf,'color','w');
subplot(4, 1, 1);
stem(n, n1_dist,'Color', 'b', 'MarkerSize', 10, 'LineWidth', 3);
set(gca,'FontSize',15);
hold on;
stem(n, n1_act_distances,'Marker', 'x', 'Color', 'r', 'MarkerSize', 10, 'LineStyle','none', 'LineWidth',4);
set(gca,'FontSize',15);
ylabel('Node 1');

subplot(4, 1, 2);
stem(n, n2_dist,'Color', 'b', 'MarkerSize', 10, 'LineWidth', 3);
set(gca,'FontSize',15);
hold on;
stem(n, n2_act_distances,'Marker', 'x', 'Color', 'r', 'MarkerSize', 10, 'LineStyle','none', 'LineWidth',4);
set(gca,'FontSize',15);
ylabel('Node 2');

subplot(4, 1, 3);
stem(n, n3_dist,'Color', 'b', 'MarkerSize', 10, 'LineWidth', 3);
hold on;
stem(n, n3_act_distances,'Marker', 'x', 'Color', 'r', 'MarkerSize', 10, 'LineStyle','none', 'LineWidth',4);
set(gca,'FontSize',15);
ylabel('Node 3');

subplot(4, 1, 4);
stem(n, n4_dist,'Color', 'b', 'MarkerSize', 10, 'LineWidth', 3);
set(gca,'FontSize',15);
hold on;
stem(n, n4_act_distances,'Marker', 'x', 'Color', 'r', 'MarkerSize', 10, 'LineStyle','none', 'LineWidth',4);
set(gca,'FontSize',15);
ylabel('Node 4');
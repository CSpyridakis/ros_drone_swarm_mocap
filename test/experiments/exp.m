clear all ; close all ; clc ; 
DEBUG = true ; dirpath = './photos' ; ext = '.jpg' ; if ~DEBUG && ~exist(dirpath,'dir') ; mkdir(dirpath); end

format bank

fontSize = 30;
titleFontSize = fontSize + 4;
axisFontSize = 20;
lineWidth = 8;

%% -----------------------------------------------------------------------------------------
%% CPU

% Init data
Data = csvread("cpu.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
cpuD = Data(:,2);

cpuD = cpuD * 100;
cpuAve = sum(cpuD)/length(cpuD) * ones(length(cpuD), 1);
cpuMax = max(cpuD) * ones(length(cpuD), 1);
cpuMin = min(cpuD) * ones(length(cpuD), 1);

f1 = figure();
p1 = plot(timeD, cpuD, timeD, cpuAve, timeD, cpuMin, timeD, cpuMax);
set(gca,'FontSize',axisFontSize);
set(p1(1), 'linewidth', 2);
set(p1(2), 'linewidth', lineWidth);
set(p1(3), 'linewidth', lineWidth);
set(p1(4), 'linewidth', lineWidth);

text(timeD(end), cpuAve(1), strcat('avg = ', int2str(cpuAve(1)), ' %'), 'FontSize', fontSize);
text(timeD(end), cpuMax(1), strcat('max = ', int2str(cpuMax(1)), ' %'), 'FontSize', fontSize);
text(timeD(end), cpuMin(1), strcat('min = ', int2str(cpuMin(1)), ' %'), 'FontSize', fontSize);

lgd = legend('Instantaneous'); set(lgd,'FontSize',fontSize);
%grid on;
title("CPU", 'FontSize', titleFontSize) ; xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('CPU usage (%)', 'FontSize', fontSize) 
if ~DEBUG ; saveas(f1,strcat(dirpath, '/', 'cpu', ext)) ; end

%% -----------------------------------------------------------------------------------------
%% DURATIONS

% Init data
Data = csvread("durations.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
durD = Data(:,3);

durD = durD * 1000;
durAve = sum(durD)/length(durD) * ones(length(durD), 1);
durMax = max(durD) * ones(length(durD), 1);
durMin = min(durD) * ones(length(durD), 1);

f2 = figure();
p2 = plot(timeD, durD, timeD, durAve, timeD, durMax, timeD, durMin);
set(gca,'FontSize',axisFontSize);
set(p2(1), 'linewidth', 2);
set(p2(2), 'linewidth', lineWidth);
set(p2(3), 'linewidth', lineWidth);
set(p2(4), 'linewidth', lineWidth); 
%grid on;

text(timeD(end), durAve(1), strcat('avg = ', int2str(durAve(1)), ' ms'), 'FontSize', fontSize);
text(timeD(end), durMax(1), strcat('max = ', int2str(durMax(1)), ' ms'), 'FontSize', fontSize);
text(timeD(end), durMin(1), strcat('min = ', int2str(durMin(1)), ' ms'), 'FontSize', fontSize);

title("HSV Calculations Duration", 'FontSize', titleFontSize) ; xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('Duration (ms)', 'FontSize', fontSize) ;
if ~DEBUG ; saveas(f2,strcat(dirpath, '/', 'durations', ext)) ; end

%% -----------------------------------------------------------------------------------------
%% RAM

% Init data
Data = csvread("ram.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
ramD = Data(:,2); 
ramT = Data(:,3);

ramD = ramD / (1024*1024*1024);
ramT = ramT / (1024*1024*1024);

f3 = figure() ;  
p3 = plot(timeD, ramD, timeD, ramT);
set(gca,'FontSize',axisFontSize); 
set(p3(1), 'linewidth', lineWidth);
set(p3(2), 'linewidth', lineWidth);

%grid on;
lgd = legend('Used','Total');  set(lgd,'FontSize',fontSize);
title("RAM", 'FontSize', titleFontSize) ; xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('Memory (GB)', 'FontSize', fontSize) ; 
if ~DEBUG ; saveas(f3,strcat(dirpath, '/', 'ram', ext)) ; end

%% -----------------------------------------------------------------------------------------
%% TEMP

% Init data
Data = csvread("temp.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
tempD = Data(:,2);

tempAve = sum(tempD)/length(tempD) * ones(length(tempD), 1);
tempMax = max(tempD) * ones(length(tempD), 1);
tempMin = min(tempD) * ones(length(tempD), 1);

f5 = figure();
p5 = plot(timeD, tempD, timeD, tempAve, timeD, tempMax, timeD, tempMin);
set(gca,'FontSize',axisFontSize);
set(p5(1), 'linewidth', 2);
set(p5(2), 'linewidth', lineWidth);
set(p5(3), 'linewidth', lineWidth);
set(p5(4), 'linewidth', lineWidth); 

text(timeD(end), tempAve(1), strcat('avg = ', int2str(tempAve(1)), ' ^{\circ}C'), 'FontSize', fontSize);
text(timeD(end), tempMax(1), strcat('max = ', int2str(tempMax(1)), '^{\circ}C'), 'FontSize', fontSize);
text(timeD(end), tempMin(1), strcat('min = ', int2str(tempMin(1)), ' ^{\circ}C'), 'FontSize', fontSize);


%grid on;
title("Temperature", 'FontSize', titleFontSize); xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('Temperature (^{\circ}C)', 'FontSize', fontSize);
if ~DEBUG ; saveas(f4,strcat(dirpath, '/', 'network', ext)) ; end


%% -----------------------------------------------------------------------------------------
%% NETWORK

% Init data
Data = csvread("network.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
netDU = Data(:,2);
netDD = Data(:,3);

netDU = netDU / (1024*8); 
netDU(netDU == 0 ) = 0.1;
netDD = netDD / (1024*8);
netDD(netDD == 0 ) = 0.1; 

f6 = figure();
semilogy(timeD, netDU);
hold on;
semilogy(timeD, netDD);
lgd = legend('Up','Down');  set(lgd,'FontSize',fontSize);
set(gca,'FontSize',axisFontSize);
%grid on;
title("Network", 'FontSize', titleFontSize); xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('Speed (kB)', 'FontSize', fontSize);
if ~DEBUG ; saveas(f4,strcat(dirpath, '/', 'network', ext)) ; end



%% -----------------------------------------------------------------------------------------
%% ANGLES

% Init data
Data = csvread("angles.csv");
timeD = Data(:,1); initTime = timeD(1); % timeD = timeD - initTime;
anglex = Data(:,2);
angley = Data(:,3);


f7 = figure();
p7 = plot(timeD, anglex, timeD, angley);
set(gca,'FontSize',axisFontSize);
set(p7, 'linewidth', 2);

lgd = legend('anglex','angley');  set(lgd,'FontSize',fontSize);
%grid on;
title("Angles", 'FontSize', titleFontSize); xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('Angle (^{\circ})', 'FontSize', fontSize);
if ~DEBUG ; saveas(f5,strcat(dirpath, '/', 'angles', ext)) ; end


%% -----------------------------------------------------------------------------------------
%% DISTANCES

% Init data
Data = csvread("distance.csv");
timeD = Data(:,1); initTime = timeD(1); % timeD = timeD - initTime;
distance = Data(:,2);

f8 = figure();
p8 = plot(timeD, distance);
set(gca,'FontSize',axisFontSize);
set(p8, 'linewidth', 2);

%grid on;
title("Distances", 'FontSize', titleFontSize); xlabel('Time (s)', 'FontSize', fontSize) ; ylabel('Distance (m)', 'FontSize', fontSize);
if ~DEBUG ; saveas(f6,strcat(dirpath, '/', 'distance', ext)) ; end
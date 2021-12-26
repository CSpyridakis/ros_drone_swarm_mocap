clear all ; close all ; clc ; 
DEBUG = true ; dirpath = './photos' ; ext = '.jpg' ; if ~DEBUG && ~exist(dirpath,'dir') ; mkdir(dirpath); end

format bank

%% -----------------------------------------------------------------------------------------
%% CPU

% Init data
Data = csvread("cpu.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
cpuD = Data(:,2);

cpuD = cpuD * 100;

f1 = figure();
plot(timeD, cpuD);
grid on;
title("CPU") ; xlabel('Time (s)') ; ylabel('CPU usage (%)') 
if ~DEBUG ; saveas(f1,strcat(dirpath, '/', 'cpu', ext)) ; end

%% -----------------------------------------------------------------------------------------
%% DURATIONS

% Init data
Data = csvread("durations.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
durD = Data(:,3);

durD = durD * 1000;

f2 = figure();
plot(timeD, durD); 
grid on;
title("HSV Calculations Duration") ; xlabel('Time (s)') ; ylabel('Duration (ms)') ;
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
plot(timeD, ramD) ; 
hold on;
plot(timeD, ramT) ;
grid on;
legend('Used','Total');
title("RAM") ; xlabel('Time (s)') ; ylabel('Memory (GB)') ; 
if ~DEBUG ; saveas(f3,strcat(dirpath, '/', 'ram', ext)) ; end

%% -----------------------------------------------------------------------------------------
%% NETWORK

% Init data
Data = csvread("network.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
netDU = Data(:,2);
netDD = Data(:,3);

netDU = netDU / (1024*8); 
netDD = netDD / (1024*8); 

f4 = figure()
semilogy(timeD, netDU);
hold on;
semilogy(timeD, netDD);
legend('Up','Down');
grid on;
title("Network"); xlabel('Time (s)') ; ylabel('Speed (kB)');
if ~DEBUG ; saveas(f4,strcat(dirpath, '/', 'network', ext)) ; end



%% -----------------------------------------------------------------------------------------
%% ANGLES

% Init data
Data = csvread("angles.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
anglex = Data(:,2);
angley = Data(:,3);


f5 = figure()
plot(timeD, anglex);
hold on;
plot(timeD, angley);
legend('anglex','angley');
grid on;
title("Angles"); xlabel('Time (s)') ; ylabel('Angles (Degrees)');
if ~DEBUG ; saveas(f5,strcat(dirpath, '/', 'angles', ext)) ; end


%% -----------------------------------------------------------------------------------------
%% DISTANCES

% Init data
Data = csvread("distance.csv");
timeD = Data(:,1); initTime = timeD(1); timeD = timeD - initTime;
distance = Data(:,3);

f6 = figure()
plot(timeD, distance);
grid on;
title("Distances"); xlabel('Time (s)') ; ylabel('Distance (m)');
if ~DEBUG ; saveas(f6,strcat(dirpath, '/', 'distance', ext)) ; end
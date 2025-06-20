%Input Data


clc;
clear;
close all;


%figure counter
n = 1;


%pull data
acc_file = "OwlSat_Acceleration_Data3";
acc_data = readmatrix(acc_file);

%time
time = acc_data(:,1);
%accerlation in x-axis
accx = acc_data(:,2);
%accerlation in y-axis
accy = acc_data(:,3);
%accerlation in z-axis
accz = acc_data(:,4);

%sampling period (s)
T = time(2) - time(1);
%sampling frequency (hz)
Fs = 1/T;


%All acceleration data
figure(n);
n = n+1;
hold on;
subplot(3,1,1);
plot(time,accx,'LineWidth',2);
%findpeaks(accx);
title('CanSat Acceleration');
xlabel('Time (Seconds)'); 
ylabel('Acceleration (m/s^2)'); 

subplot(3,1,2);
plot(time,accy,'LineWidth',2);
title('CanSat Acceleration');
xlabel('Time (Seconds)'); 
ylabel('Acceleration (m/s^2)'); 

subplot(3,1,3);
plot(time,accz,'LineWidth',2);
title('CanSat Acceleration');
xlabel('Time (Seconds)'); 
ylabel('Acceleration (m/s^2)'); 


%frequency response
figure(n);
n = n+1;

%find the [value of the peaks, indice they occur]
[xpeak, xloc] = findpeaks(accx);

for i = 1:length(xloc)-1
    %determine indice difference between peaks
    diffx(i) = xloc(i+1) - xloc(i);
    %determine time between peaks
    cros_timex(i) = diffx(i) * T;
    %determine frequency
    freqx(i) = 1/cros_timex(i);
end


%find the [value of the peaks, indice they occur]
[ypeak, yloc] = findpeaks(accy);

for j = 1:length(yloc)-1
    %determine indice difference between peaks
    diffy(j) = yloc(j+1) - yloc(j);
    %determine time between peaks
    cros_timey(j) = diffy(j) * T;
    %determine frequency
    freqy(j) = 1/cros_timey(j);
end

%find the [value of the peaks, indice they occur]
[zpeak, zloc] = findpeaks(accz);

for k = 1:length(zloc)-1
    %determine indice difference between peaks
    diffz(k) = zloc(k+1) - zloc(k);
    %determine time between peaks
    cros_timez(k) = diffz(k) * T;
    %determine frequency
    freqz(k) = 1/cros_timez(k);
end

%plot first frequncy
subplot(3,1,1);
plot(1:i,freqx,'LineWidth',2);
title('CanSat Frequency Response');
xlabel("Time")
ylabel("Frequency")

%plot first frequncy
subplot(3,1,2);
plot(1:j,freqy,'LineWidth',2);
title('CanSat Frequency Response');
xlabel("Time")
ylabel("Frequency")

%plot first frequncy
subplot(3,1,3);
plot(1:k,freqz,'LineWidth',2);
title('CanSat Frequency Response');
xlabel("Time")
ylabel("Frequency")


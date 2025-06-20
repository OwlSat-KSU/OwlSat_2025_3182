%Input Data


clc;
clear;
close all;


%figure counter
n = 1;


%pull data
acc_file = "OwlSat_Acceleration_Data2";
acc_data = readmatrix(acc_file);

%sampling frequency (hz)
Fs = 20;
%sampling Period 
T = 1/Fs;

%time
time_all = acc_data(:,1);
%accerlation in x-axis
accx_all = acc_data(:,2);
%accerlation in y-axis
accy_all = acc_data(:,3);
%accerlation in z-axis
accz_all = acc_data(:,4);

%Window for times
window_start = 0.05;
window_end = 13.25;

%constrain data
time = time_all(find(time_all == window_start) : find(time_all == window_end));
accx = accx_all(find(time_all == window_start) : find(time_all == window_end));
accy = accy_all(find(time_all == window_start) : find(time_all == window_end));
accz = accz_all(find(time_all == window_start) : find(time_all == window_end));



%All acceleration data
figure(n);
n = n+1;
hold on;
plot(time,accx,'LineWidth',2);
plot(time,accy,'LineWidth',2);
plot(time,accz,'LineWidth',2);
title('CanSat Acceleration');
xlabel('Time (Seconds)'); 
ylabel('Acceleration (m/s^2)'); 
grid;
axis padded;




%fourier Transforms
fx = fft(accx);
fy = fft(accy);
fz = fft(accz);

%grab length of plotted signal
L = length(fx);

figure(n);
n = n+1;

%plot first transform
subplot(3,1,1);
plot(Fs/L*(0:L-1),abs(fx),'LineWidth',2);
title('CanSat Frequency Response');
xlabel("f (Hz)")
ylabel("|fft(X)|")


%plot first transform
subplot(3,1,2);
plot(Fs/L*(0:L-1),abs(fy),'LineWidth',2);
title('CanSat Frequency Response');
xlabel("f (Hz)")
ylabel("|fft(X)|")


%plot first transform
subplot(3,1,3);
plot(Fs/L*(0:L-1),abs(fz),'LineWidth',2);
title('CanSat Frequency Response');
xlabel("f (Hz)")
ylabel("|fft(X)|")


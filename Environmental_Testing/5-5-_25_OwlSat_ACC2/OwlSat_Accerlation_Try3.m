%Input Data


clc;
clear;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%choose which data set to analyize
selector = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%pull data
if selector == 1
    acc_file = "OwlSat_ACC1";
    c1 = 2;
    c2 = 3;
    c3 = 4;
elseif selector == 2
    acc_file = "OwlSat_ACC1";
    c1 = 2+9*(selector - 1); %11
    c2 = 3+9*(selector - 1); %12
    c3 = 4+9*(selector - 1); %13
elseif selector == 3
    acc_file = "OwlSat_ACC1";
    c1 = 2+9*(selector - 1); %20;
    c2 = 3+9*(selector - 1);
    c3 = 4+9*(selector - 1);
elseif selector == 4
    acc_file = "OwlSat_ACC1";
    c1 = 2+9*(selector - 1);
    c2 = 3+9*(selector - 1);
    c3 = 4+9*(selector - 1);
elseif selector == 5
    acc_file = "OwlSat_ACC1";
    c1 = 2+9*(selector - 1);
    c2 = 3+9*(selector - 1);
    c3 = 4+9*(selector - 1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%load in data
acc_data = readmatrix(acc_file);

%time
time = acc_data(:,1);
%accerlation in x-axis
acc{1} = acc_data(:,c1);
%accerlation in y-axis
acc{2} = acc_data(:,c2);
%accerlation in z-axis
acc{3} = acc_data(:,c3);

%sampling period (s)
T = time(2) - time(1);
%sampling frequency (hz)
Fs = 1/T;


%figure counter
n = 1;


%All acceleration data
figure(n);
n = n+1;
hold on;

for i = 1:3
    subplot(3,1,i);
    plot(time,acc{i},'LineWidth',0.25);
    %findpeaks(accx);
    if i == 1
        title('CanSat X-Acceleration');
    elseif i == 2
        title('CanSat Y-Acceleration');
    elseif i == 3
        title('CanSat Z-Acceleration');
    end
    xlabel('Time (Seconds)'); 
    ylabel('Acceleration (m/s^2)'); 
end

%frequency plot
figure(n);
n = n+1;

for i = 1:3

    %find the [value of the peaks, indice they occur]
    [peak{i}, loc{i}] = findpeaks(acc{i});
    %find length of indice vector
    counting{i} = length(loc{i});

    for j = 1:(counting{i}-1)
        %determine indice difference between peaks
        diff{i}(j) = loc{i}(j+1) - loc{i}(j);
        %determine time between peaks
        cros_time{i}(j) = diff{i}(j) * T;
        %determine frequency
        freq{i}(j) = 1/cros_time{i}(j);
        %remap time values to size of peaks
        aug_time{i}(j) = time(loc{i}(j));
    end

end


%plot all the frequency responses
for i = 1:3
    subplot(3,1,i);
    plot(aug_time{i},freq{i},'LineWidth',0.25);
    if i == 1
        title('CanSat X-Frequency Response');
    elseif i == 2
        title('CanSat Y-Frequency Response');
    elseif i == 3
        title('CanSat Z-Frequency Response');
    end
    xlabel("Time (s)");
    ylabel("Frequency (Hz)");
end


%frequency distribution plot
figure(n);
n = n+1;

%determine the distribution of frequenices???
for i = 1:3
    %Organize distribution
    [freq_org{i}, freq_mag{i}] = unique(freq{1});
    %plot with bars
    subplot(3,1,i);
    bar(freq_org{i},freq_mag{i},'LineWidth',0.25);
    %titles
    if i == 1
        title('CanSat X-Frequency Distribution');
    elseif i == 2
        title('CanSat Y-Frequency Distribution');
    elseif i == 3
        title('CanSat Z-Frequency Distribution');
    end
    xlabel("Frequency (Hz)");
    ylabel("Count");
    axis padded;

    
end



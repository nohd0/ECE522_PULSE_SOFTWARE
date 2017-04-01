%% Wireless Cuff Blood Pressure Detection
%  Created by Evan Williams 3/26/17
%  This script utilizes the dataset located at 
%  https://archive.ics.uci.edu/ml/datasets/Cuff-Less+Blood+Pressure+Estimation#
%  to develop the signal processing required for our cuffless blood
%  pressure estimator wearable that we are developing for medical
%  instrumentation taught by Dr. Nagle. 

%  Attribute Information: (information about the dataset from the website)
% 
%  The data set is in matlab's v7.3 mat file, accordingly it should be opened
%  using new versions of matlab or HDF libraries in other environments.
%  (Please refer to the Web for more information about this format) 
%  This database consist of a cell array of matrices, each cell is one record part. 
%  In each matrix each row corresponds to one signal channel: 
%  1: PPG signal, FS=125Hz; photoplethysmograph from fingertip 
%  2: ABP signal, FS=125Hz; invasive arterial blood pressure (mmHg) 
%  3: ECG signal, FS=125Hz; electrocardiogram from channel II

% load one portion of the dataset into MATLAB and parse it into the RawData
% matrix.
clc, clear all, close all
A = load('Part_1.mat');
RawData = A.Part_1;

%%
close all; % keeps too many plots from appearing at once.
fs = 125; % sampling rate in Hz
Fs = 1/fs; % sampling period in seconds
n1 = numel(A); % number of cells extracted
MatrixData = []; % Blank Matrix for the extraction of data from the cells
x = 1;
for i = 1:x % x cannot be greater than n1 will fix this with code later
    MatrixData = [MatrixData RawData{1,i}];
end
PPG = MatrixData(1,:); % PPG Signal
ABP = MatrixData(2,:); % Blood Pressure Signal
ECG = MatrixData(3,:); % ECG Signal
t = 0:Fs:(length(PPG)-1)*Fs; % constructed time vector
shorterdata = round(1/80*length(t)); % smaller subset of the dataset to make it easier to visualize

figure();
plot(t, PPG, t, ABP, t, ECG)
xlabel('time (s)');
title('Tiny subset of dataset');
legend('PPG Signal', 'Arterial Blood Pressure', 'ECG Signal')
figure();
plot(t(1,1:shorterdata), ECG(1,1:shorterdata))
xlabel('time (s)')
title('ECG Data')
ylabel('ECG Signal');
figure();
plot(t(1,1:shorterdata), PPG(1,1:shorterdata))
xlabel('time (s)')
title('PPG Data')
ylabel('PPG Signal');

% I used the pan tompkin's algorithm implementation available at 
% https://www.mathworks.com/matlabcentral/fileexchange/45840-complete-pan-tompkins-implementation-ecg-qrs-detector
% All credit goes to Hooman Sedghamiz the programmer responsible for this
% implementation.
[ECGval, ECGind, delay] = pan_tompkin(ECG(1,1:shorterdata), fs, 1);


%% Extraction of the PPG data
% close all;
PPGShort = PPG(1,1:shorterdata);
[PPGval, PPGind] = PPGSigProcess(PPGShort, fs, 1);

%% Implementation of the Systolic Blood Pressure Estimation
% This will need some more work later but it follows the paper:
% Cuff-Free Blood Pressure Estimation Using Pulse Transit Time and Heart Rate

% Make sure the vectors are the same length in the case that the data
% truncates the peak of the next 
if length(PPGind) < length(ECGind)
    ECGind = ECGind(1, 1:length(PPGind));    
elseif length(PPGind) > length(ECGind)
    PPGind = PPGind(1, 1:length(ECGind));    
end

%might need to revisit this later
PTT = abs(ECGind - PPGind).*Fs; % The pulse transit time (units of seconds) will probably need to revise later
HR = diff(ECGind).*Fs.*60; % Basic HR Estimation (beats/min) will need to revise later
BP = zeros(1,length(PTT)-1);
BP(1) = 60; % initial value for Blood Pressure needs to be updated later
% constants for equation. I will update these later
a = 1;
b = 1;
c = 1;
d = 1;
for i = 1:length(PTT)-1
   BP(i+1) = a*log(PTT(i))+b*HR(i)+c*BP(i)+d;
end

BP = BP(1, 2:length(BP)); % Calculated Systolic Blood Pressure



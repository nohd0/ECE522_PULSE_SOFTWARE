function [PPGraw, PPGindex] = PPGSigProcess(PPG, fs, gr)
%% This section includes the implementation of the PPG Signal Processing Algorithm
% The PPGSigProcess function locates peaks in the PPG signal and
% returns the raw amplitude of the PPG signal as well as the indices of 
% these these points. 

% Taking the Derivatives of the PPG Signal
Fs = 1/fs; % sampling period in seconds
t = 0:Fs:(length(PPG)-1)*Fs; % constructed time vector
PPG_1stDeriv = diff(PPG)./diff(t);
PPG_2ndDeriv = diff(PPG_1stDeriv)./diff(t(1,1:length(PPG_1stDeriv)));

% Find the second derivative zeros
a = diff(sign(PPG_2ndDeriv));
PPGindex = find(a~=0);
PPGraw = PPG(PPGindex);

% Find the First Derivative Zeros
b = diff(sign(PPG_1stDeriv));
PPGindex2 = find(b~=0);

% Filtered Signal Peak Detection
Wn = 12*2/fs;
N = 3; % order of 3 less processing
[c,d] = butter(N,Wn,'low'); %bandpass filtering
PPGsq = PPG.^2;
PPG_l = filtfilt(c,d,PPGsq);
PPG_l = PPG_l/ max(abs(PPG_l));
PPGFilt1stDeriv = diff(PPG_l)./diff(t);
e = diff(sign(PPGFilt1stDeriv));
PPGindex3 = find(e~=0);

% MATLAB findpeaks algorithm
[PPGVal4, PPGindex4] = findpeaks(PPG);

% If the graphics flag is set to one then plots of the data will be
% displayed
if gr == 1
    
    % Plot the 2nd Derivative
    figure();
    plot(t(1,1:length(PPG_2ndDeriv)), PPG_2ndDeriv)
    title('2nd Derivative of PPG')
    xlabel('time (s)');
    ylabel('PPG Signal Amplitude');
    
    % Plot PPG signal and detected peaks
    figure();
    plot(t, PPG, 'b');
    hold on
    plot(t(PPGindex), PPG(PPGindex), 'or');
    xlabel('time (s)');
    ylabel('PPG Signal Amplitude');
    legend('PPG', 'Detected Peaks')
    title('Detected Peaks 2nd Deriv');
    
    % Plot PPG signal and detected peaks
    figure();
    plot(t, PPG, 'b');
    hold on
    plot(t(PPGindex2), PPG(PPGindex2), 'or');
    hold off;
    xlabel('time (s)');
    ylabel('PPG Signal Amplitude');
    legend('PPG', 'Detected Peaks')
    title('Detected Peaks 1st Deriv');
    
    % Plot filtered PPG Signal
    figure();
    plot(t, PPG_l, t, PPG);
    legend('Filtered PPG', '#NoFilter')
    xlabel('time (s)');
    ylabel('PPG Signal Amplitude');
    
    % Plot detected peaks from filtered signal
    figure();
    plot(t, PPG, 'b');
    hold on;
    plot(t(PPGindex3), PPG(PPGindex3), 'or');
    hold off;
    title('Detected Peaks w/ Filtered Data');
    xlabel('time (s)');
    ylabel('PPG Signal Amplitude');
    
    % Plot detected peaks from MATLAB function
    figure();
    plot(t, PPG, 'b');
    hold on;
    plot(t(PPGindex4), PPG(PPGindex4), 'or');
    hold off;
    title('Detected Peaks w/ MATLAB findpeaks');
    xlabel('time (s)');
    ylabel('PPG Signal Amplitude');
    
end

PPGindex = PPGindex4; % Use the findpeaks method
PPGraw = PPG(PPGindex); % Extract the values from the PPG method

end
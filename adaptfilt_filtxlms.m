clear all
clc

Fs = 8000;
N = 8000;
x  = randn(1,N);    % Noise source
g  = fir1(47,0.4);       % FIR primary path system model
n  = 0.1*randn(1,N);  % Observation noise signal
d  = filter(g,1,x)+n;    % Signal to be cancelled
b  = fir1(31,0.4);       % FIR secondary path system model
mu = 0.008;              % Filtered-X LMS step size
lms = dsp.FilteredXLMSFilter(32, 'StepSize', mu, 'LeakageFactor', ...
     1, 'SecondaryPathCoefficients', b);
[y,e] = step(lms,x,d);
plot(1:N,d,1:N,y,1:N,e);
title('Active Noise Control of a Random Noise Signal');
legend('Desired','Output','Error');
xlabel('Time Index'); ylabel('Signal Value');  grid on;
hold on

ratio = abs(e./d);
dB = 20*log10(ratio);
figure
plot(1:N,dB,1:N,zeros(1,N))

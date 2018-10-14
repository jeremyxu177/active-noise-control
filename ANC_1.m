Fs     = 8e3;  % 8 kHz
N      = 800;  % 800 samples@8 kHz = 0.1 seconds
Flow   = 160;  % Lower band-edge: 160 Hz
Fhigh  = 2000; % Upper band-edge: 2000 Hz
delayS = 7;
Ast    = 20;   % 20 dB stopband attenuation
Nfilt  = 8;    % Filter order

% Design bandpass filter to generate bandlimited impulse response
Fd = fdesign.bandpass('N,Fst1,Fst2,Ast',Nfilt,Flow,Fhigh,Ast,Fs);
Hd = design(Fd,'cheby2','FilterStructure','df2tsos',...
    'SystemObject',true);

% Filter noise to generate impulse response
H = step(Hd,[zeros(delayS,1); log(0.99*rand(N-delayS,1)+0.01).* ...
    sign(randn(N-delayS,1)).*exp(-0.01*(1:N-delayS)')]);
H = H/norm(H);

t = (1:N)/Fs;
plot(t,H,'b');
xlabel('Time [sec]');
ylabel('Coefficient value');
title('True Secondary Path Impulse Response');
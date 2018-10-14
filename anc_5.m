delayW = 15;
Flow   = 200; % Lower band-edge: 200 Hz
Fhigh  = 800; % Upper band-edge: 800 Hz
Ast    = 20;  % 20 dB stopband attenuation
Nfilt  = 10;  % Filter order

% Design bandpass filter to generate bandlimited impulse response
Fd2 = fdesign.bandpass('N,Fst1,Fst2,Ast',Nfilt,Flow,Fhigh,Ast,Fs);
Hd2 = design(Fd2,'cheby2','FilterStructure','df2tsos',...
    'SystemObject',true);

% Filter noise to generate impulse response
G = step(Hd2,[zeros(delayW,1); log(0.99*rand(N-delayW,1)+0.01).*...
    sign(randn(N-delayW,1)).*exp(-0.01*(1:N-delayW)')]);
G = G/norm(G);

plot(t,G,'b');
xlabel('Time [sec]');
ylabel('Coefficient value');
title('Primary Path Impulse Response');
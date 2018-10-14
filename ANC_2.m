ntrS = 30000;
s = randn(ntrS,1); % Synthetic random signal to be played
Hfir = dsp.FIRFilter('Numerator',H.');
dS = step(Hfir,s) + ... % random signal propagated through secondary path
    0.01*randn(ntrS,1); % measurement noise at the microphone

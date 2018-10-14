% FIR Filter to be used to model primary propagation path
Hfir = dsp.FIRFilter('Numerator',G.');

% Filtered-X LMS adaptive filter to control the noise
L = 350;
muW = 0.0001;
Hfx = dsp.FilteredXLMSFilter('Length',L,'StepSize',muW,...
    'SecondaryPathCoefficients',Hhat);

% Sine wave generator to synthetically create the noise
A = [.01 .01 .02 .2 .3 .4 .3 .2 .1 .07 .02 .01]; La = length(A);
F0 = 60; k = 1:La; F = F0*k;
phase = rand(1,La); % Random initial phase
Hsin = dsp.SineWave('Amplitude',A,'Frequency',F,'PhaseOffset',phase,...
    'SamplesPerFrame',512,'SampleRate',Fs);

% Audio player to play noise before and after cancellation
Hpa = audioDeviceWriter('SampleRate',Fs);

% Spectrum analyzer to show original and attenuated noise
Hsa = dsp.SpectrumAnalyzer('SampleRate',Fs,'OverlapPercent',80,...
    'SpectralAverages',20,'PlotAsTwoSidedSpectrum',false,...
    'ShowLegend',true, ...
    'ChannelNames', {'Original noisy signal', 'Attenuated noise'});
clear all
close all

% T = 1000; %simulation duration
fs=8000;
t=0:1/fs:1;
T=length(t)-1;

Pw=[0.9 -0.3 0.7 -0.28 0.5 -0.25 0.36 -0.22 0.3 -0.18 0.28 -0.15 0.22 -0.1 0.2 -0.1 0.14 -0.08 0.1 -0.05 0.05 -0.03 0.02 -0.01 0.01 -0.01];
% Pw = exp(-(1:length(P))).*P;
% Sw = fir1(13,0.3);
Sw=Pw(13:end)*0.5;

%% assume Sw = s_hat_w
L = 16; %filter order
X = randn(1,T); % define the input noise(source signal)
d = filter(Pw,1,X); % desired signal (signal at the error microphone)
x_hat = filter(Sw,1,X);

% Initialization of Active Noise Control
Cx = zeros(1,L); %reference signal vector of W(z)
Cw = zeros(1,L); %coefficient vector of W(z)
X_s = zeros(1,L);
e = zeros(1,T); %define error value


mu = 0.1
for n = 1:T
    Cx = [x_hat(n) Cx(1:L-1)];
    y = sum(Cx.*Cw);
    e(n) = d(n) - y;
    X_s = [x_hat(n) X_s(1:L-1)];
    Cw = Cw + mu*e(n)*X_s;
end
figure
subplot(2,1,1)
plot([1:T],e)
ylabel('Amplitude');
xlabel('Discrete time n');
legend('Noise residual');

subplot(2,1,2)
plot([1:T],d)
hold on
plot([1:T],d-e,'r:') %d-e right; y wrong ????;
ylabel('Amplitude');
xlabel('Discrete time n');
legend('Noise signal','Control signal');

ratio = abs(e.^2)./abs(d.^2);
dB = 20*log10(ratio);
figure
plot(1:T,dB,'.',1:T,zeros(1,T))
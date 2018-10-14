clear all
close all

T = 8000; %simulation duration

% set primary path
% Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01 0.02 0.01];
% Pw = [0.01 0.25 0.5 1 0.5 0.25 0.01 0.02 0.01 0.3 0.5];
Pw=[0.9 -0.7 0.8 -0.48 0.5 -0.35 0.36 -0.32 0.3 -0.22 0.28 -0.2...
    0.22 -0.15 0.2 -0.14 0.14 -0.08 0.1 -0.05 0.05 -0.03 0.02 -0.01];
% secondary path
% Sw=Pw(3:7)*0.25;
% Sw=Pw(3:end)*0.25;
Sw=Pw(13:end)*0.6;
% Sw=Pw*0.25;
%% secondary path estimation (offline modelling)
x_iden = randn(1,T); % generate a white noise
% send it to the error microphone, i.e. the identification error
y_iden = filter(Sw,1,x_iden);
% y_iden = conv(Sw,x_iden);

%% offline modelling
L = 16; %filter order, order should be sufficient to accurately model the response...
%of the physical system. The larger, the better???? This value should be
%larger than length(Pw)
% L =30; % accuracy decreases
% L = 20;
S_hat_x = zeros(1,L); %define the reference signal vector of  S(z)_hat
S_hat_w = zeros(1,L); %define the coefficient vector of the S(z)_hat
e_iden = zeros(1,L); %define the identification error in the estimation process of...
% the seconday path

% apply LMS algorithm
% mu = 0.1;
% mu = 0.05; % estimation accuracy decreases
mu = 0.1; %also a very good value, performs better than 0.1
for n = 1:T %time index
    S_hat_x = [x_iden(n) S_hat_x(1:L-1)]; %update the reference signal vector
    Shy = sum(S_hat_x.*S_hat_w); % calculate output of Secondary path estimate
    e_iden(n)=y_iden(n)-Shy;    % calculate error         
    S_hat_w=S_hat_w + mu*e_iden(n)*S_hat_x;   % adjust the weight
end

%%plotting
subplot(2,1,1)
plot([1:T], e_iden)
ylabel('Amplitude');
xlabel('Discrete time n');
legend('Identification error');
subplot(2,1,2)
stem(Sw) 
hold on 
stem(S_hat_w, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of Secondary Path', 'Coefficients of Secondary Path Estimate')

%% Then we can propagate the seconday path estimate into the entire system
% X=cos(2*pi*(1/5)*[0:T-1]); %feedback can only deal with narrowband
% X=0.9*cos(2*pi*(1/4)*[0:T-1]);
%When the input is narrowband only, the performance is extremely better
%than (narowband + broadband).
X=0.03*randn(1,T)+0.9*cos(2*pi*(1/5)*[0:T-1]);
% X=0.1*randn(1,T);
d = filter(Pw,1,X); % desired signal (signal at the error microphone)

% Since we do not have any reference signal, we first define a vector for
% reference signal
x_ref = zeros(size(X));

% Initialization of Active Noise Control
Cx = zeros(1,L); %reference signal vector of W(z)
Cw = zeros(1,L); %coefficient vector of W(z)
Cyx = zeros(1,L); %adaptive filter output estimate???
Sx = zeros(size(Sw)); %???
e = zeros(1,T); %define error value
X_s = zeros(1,L); %filtered X

%% send the first sample of control signal
n = 1;
Cx = [x_ref(n) Cx(1:L-1)];
Cy = sum(Cx.*Cw); %adaptive filter output
Sx = [Cy Sx(1:length(Sx)-1)];
e(n) = d(n) - sum(Sx.*Sw);
%%
mu = 0.15;
for n = 2:T
    Cyx = [Cy Cyx(1:L-1)];
    x_ref(n) = e(n-1) + sum(Cyx.*S_hat_w);
    
    Cx = [x_ref(n) Cx(1:L-1)];
    Cy = sum(Cx.*Cw);
    Sx = [Cy Sx(1:length(Sx)-1)];
    e(n) = d(n) - sum(Sx.*Sw);
    
    S_hat_x  = [X(n) S_hat_x(1:L-1)]; %update the vector signal of secondary path estimate
    X_s = [sum(S_hat_x.*S_hat_w) X_s(1:L-1)]; %i.e. x' in the paper
    Cw = Cw + mu*e(n)*X_s; %update the coefficiet vector of adaptive filter
end

% %Plotting
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

% ratio = abs(e./d);
% dB = 20*log10(ratio);
% figure
% plot(1:T,dB,'.',1:T,zeros(1,T))
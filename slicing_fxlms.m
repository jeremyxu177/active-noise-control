clear all
close all

% T = 1000; %simulation duration
fs=8000;
t=0:1/fs:1;
T=length(t);
% set primary path
% Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
% Sw=Pw*0.25;
% % Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01 0.02 0.01];
% P = [0.01 0.25 0.5 1 0.5 0.25 0.01 0.02 0.01 0.3 0.5];
% Pw = exp(-(1:length(P))).*P;
% % secondary path
% % Sw=Pw(3:7)*0.25;
% Sw=Pw(3:end)*0.25;

Pw=[0.9 -0.3 0.7 -0.28 0.5 -0.25 0.36 -0.22 0.3 -0.18 0.28 -0.15 0.22 -0.1 0.2 -0.1 0.14 -0.08 0.1 -0.05 0.05 -0.03 0.02 -0.01 0.01 -0.01];
% Pw = exp(-(1:length(P))).*P;
% Sw = fir1(13,0.3);
Sw=Pw(13:end)*0.6;
%% secondary path estimation (offline modelling)
x_iden = rand(1,T); % generate a white noise

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
mu = 0.2; %also a very good value, performs better than 0.1
for n = 1:T %time index
    S_hat_x = [x_iden(n) S_hat_x(1:L-1)]; %update the reference signal vector
    Shy = sum(S_hat_x.*S_hat_w); % calculate output of Secondary path estimate
    e_iden(n)=y_iden(n)-Shy;    % calculate error         
    S_hat_w=S_hat_w + mu*e_iden(n)*S_hat_x;   % adjust the weight
end

%plotting
figure
subplot(2,1,1)
plot([1:T], e_iden)
title('secondary path offline modelling');
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
X = randn(1,T)+ 0.8*randn(1,T); % define the input noise(source signal)
% X=0.9*cos(2*pi*(1/5)*[0:T-1]);
% X = cos(2*pi*(1/5)*[0:T-1]);
d = filter(Pw,1,X); % desired signal (signal at the error microphone)
% Initialization of Active Noise Control
Cx = zeros(1,L); %reference signal vector of W(z)
Cw = zeros(1,L); %coefficient vector of W(z)
Sx = zeros(size(Sw)); %???
e = zeros(1,T); %define error value
X_s = zeros(1,L); %filtered X

%% Apply FXLMS
mu = 0.1;
for n = 1:T %time index
    Cx = [X(n) Cx(1:L-1)]; %update the reference signal vector
    Cy = sum(Cx.*Cw); %Adaptive filter output
    Sx = [Cy Sx(1:length(Sx)-1)]; %propagate to secondary path
    y = sum(Sx.*Sw); %output passing through secondary path, to the reference microphone
    e(n) = d(n) - y; %calculating the residual error
    S_hat_x  = [X(n) S_hat_x(1:L-1)]; %update the vector signal of secondary path estimate
    X_s = [sum(S_hat_x.*S_hat_w) X_s(1:L-1)]; %i.e. x' in the paper
    Cw = Cw + mu*e(n)*X_s; %update the coefficiet vector of adaptive filter
end

%Plotting
figure
subplot(2,1,1)
plot([1:T],e)
title('ANC time domain performance');
ylabel('Amplitude');
xlabel('Discrete time n');
legend('Noise residual');

subplot(2,1,2)
plot(1:T,d,1:T,d-e,'r:') %d-e right; y wrong ????;
ylabel('Amplitude');
xlabel('Discrete time n');
legend('Noise signal','Control signal');

figure
interval = [(1:0.25*T);(0.25*T:0.5*T-1) ;(0.5*T:0.75*T-1); (0.75*T:T-1)];
for i = 1:4
    F1 = fft(d(interval(i,:)))./length(interval(i,:));
    df = (1/T)*fs;
    f = df*interval(1,:);
%     figure
    subplot(4,3,i*3-2)
    plot(f,abs(F1))
    
    
    F2 = fft(e(interval(i,:)))./length(interval(i,:));
    df = (1/T)*fs;
    f = df*interval(1,:);
    %figure
    subplot(4,3,3*i-1)
    plot(f,abs(F2))
    
    ratio = abs(F2./F1);
    dB = 20*log10(ratio);
    subplot(4,3,3*i)
    plot(f,dB,f,zeros(1,length(interval(i,:))));
end

%% TIME DOMAIN POWER ANALYSIS
power_sum_e = zeros(1,T);
power_sum_d = zeros(1,T);
ratio3 = zeros(1,T);
power_ratio = zeros(1,T);
for n = 1:T
    power_e = abs(e.^2);
    power_d = abs(d.^2);
    power_sum_e = power_sum_e + power_e;
    power_sum_d = power_sum_d + power_d;
    ratio3 = power_sum_e./power_sum_d;
    power_ratio = 10*log10(ratio3);
end
% disp(ratio3)
% disp(power_ratio)
hold on
figure
plot([1:T],power_ratio)
title('power ratio decline in time domain');

% figure
% F1=fft(X(1:0.25*T))./[1:0.25*T];
% df=(1/T)*fs;
% f=df*(0:(0.25*T-1));
% subplot(2,1,1)
% % plot(f,abs(F1))
% plot(abs(F1))
% title('FFT of Noise signal');
% hold on
% 
% F2=fft(e(1:0.25*T))./[1:0.25*T];
% df=(1/T)*fs;
% f=df*(0:(0.25*T-1));
% subplot(2,1,2)
% % plot(f,abs(F2))
% plot(abs(F2))
% title('FFT of Residual signal');
% hold on
% 
% ratio1 = abs(e(1:0.25*T)./d(1:0.25*T));
% ratio2 = abs(F2(1:0.25*T)./F1(1:0.25*T));
% dB = 20*log10(ratio2);
% figure
% % plot(f,dB,f,zeros(1,T))
% plot(dB)
% title('ANC frequency domain performance');
% hold on
% 
% figure
% F1=fft(X(0.25:0.5*T))./[0.25:0.5*T];
% df=(1/T)*fs;
% f=df*(0.25*T:(0.5*T-1));
% subplot(2,1,1)
% % plot(f,abs(F1))
% plot(abs(F1))
% title('FFT of Noise signal');
% hold on
% 
% F2=fft(e(0.25:0.5*T))./[0.25:0.5*T];
% df=(1/T)*fs;
% f=df*(0.25*T:(0.5*T-1));
% subplot(2,1,2)
% % plot(f,abs(F2))
% plot(abs(F2))
% title('FFT of Residual signal');
% hold on
% 
% ratio1 = abs(e(0.25:0.5*T)./d(0.25:0.5*T));
% ratio2 = abs(F2(0.25:0.5*T)./F1(0.25:0.5*T));
% dB = 20*log10(ratio2);
% figure
% % plot(f,dB,f,zeros(1,T))
% plot(dB)
% title('ANC frequency domain performance');
% hold on
% 
% figure
% F1=fft(X(0.5:0.75*T))./[0.5:0.75*T];
% df=(1/T)*fs;
% f=df*(0.5*T:(0.75*T-1));
% subplot(2,1,1)
% % plot(f,abs(F1))
% plot(abs(F1))
% title('FFT of Noise signal');
% hold on
% 
% F2=fft(e(0.5:0.75*T))./[0.5:0.75*T];
% df=(1/T)*fs;
% f=df*(0.5:(0.75*T-1));
% subplot(2,1,2)
% % plot(f,abs(F2))
% plot(abs(F2))
% title('FFT of Residual signal');
% hold on
% 
% ratio1 = abs(e(0.5:0.75*T)./d(0.5:0.75*T));
% ratio2 = abs(F2(0.5:0.75*T)./F1(0.5:0.75*T));
% dB = 20*log10(ratio2);
% figure
% % plot(f,dB,f,zeros(1,T))
% plot(dB)
% title('ANC frequency domain performance');
% hold on
% %% FREQUENCY DOMAIN
% figure
% subplot(6,2,1)
% plot(abs(F1(1:0.25*T)));
% title('first quarter of iterations');
% subplot(6,2,3)
% plot(abs(F2(1:0.25*T)));
% subplot(6,2,5)
% plot(dB(1:0.25*T));
% 
% subplot(6,2,2)
% plot(abs(F1(0.25*T:0.5*T)));
% title('second quarter of iterations');
% subplot(6,2,4)
% plot(abs(F2(0.25*T:0.5*T)));
% subplot(6,2,6)
% plot(dB(0.25*T:0.5*T));
% 
% subplot(6,2,7)
% plot(abs(F1(0.5*T:0.75*T)));
% title('third quarter of iterations');
% subplot(6,2,9)
% plot(abs(F2(0.5*T:0.75*T)));
% subplot(6,2,11)
% plot(dB(0.5*T:0.75*T));
% 
% subplot(6,2,8)
% plot(abs(F1(0.75*T:T)));
% title('fourth quarter of iterations');
% subplot(6,2,10)
% plot(abs(F2(0.75*T:T)));
% subplot(6,2,12)
% plot(dB(0.75*T:T));
% %% TIME DOMAIN
% figure
% subplot(6,2,1)
% plot([1:0.25*T],e(1:0.25*T));
% title('first quarter of iterations');
% axis([0 8000 -5 5])
% subplot(6,2,3)
% plot([1:0.25*T],d(1:0.25*T));
% axis([0 8000 -10 10])
% subplot(6,2,5)
% plot([1:0.25*T],ratio1(1:0.25*T));
% 
% subplot(6,2,2)
% plot([0.25*T:0.5*T],e(0.25*T:0.5*T));
% title('second quarter of iterations');
% axis([0 8000 -5 5])
% subplot(6,2,4)
% plot([0.25*T:0.5*T],d(0.25*T:0.5*T));
% axis([0 8000 -10 10])
% subplot(6,2,6)
% plot([0.25*T:0.5*T],ratio1(0.25*T:0.5*T));
% 
% subplot(6,2,7)
% plot([0.5*T:0.75*T],e(0.5*T:0.75*T));
% title('third quarter of iterations');
% axis([0 8000 -5 5])
% subplot(6,2,9)
% plot([0.5*T:0.75*T],d(0.5*T:0.75*T));
% axis([0 8000 -10 10])
% subplot(6,2,11)
% plot([0.5*T:0.75*T],ratio1(0.5*T:0.75*T));
% 
% subplot(6,2,8)
% plot([0.75*T:T],e(0.75*T:T));
% title('fourth quarter of iterations');
% axis([0 8000 -5 5])
% subplot(6,2,10)
% plot([0.75*T:T],d(0.75*T:T));
% axis([0 8000 -10 10])
% subplot(6,2,12)
% plot([0.75*T:T],ratio1(0.75*T:T));
% 
% 
% %% TIME DOMAIN POWER ANALYSIS
% power_sum_e = zeros(1,T);
% power_sum_d = zeros(1,T);
% ratio3 = zeros(1,T);
% power_ratio = zeros(1,T);
% for n = 1:T
%     power_e = abs(e.^2);
%     power_d = abs(d.^2);
%     power_sum_e = power_sum_e + power_e;
%     power_sum_d = power_sum_d + power_d;
%     ratio3 = power_sum_e./power_sum_d;
%     power_ratio = 10*log10(ratio3);
% end
% disp(ratio3)
% disp(power_ratio)
% hold on
% figure
% plot([1:T],power_ratio)
% title('power_ratio decline in time domain');
%--------------------------------------------------------------------------
% I am very busy lately with my work so, regretfully, I have no choice 
% except to postpone my plan to write a new code for simulating a multi-
% channel active noise control system. In the meantime, I compose one code 
% about feed-back active noise control system (FbLMS), which is somewhat 
% different to the feed-forward system (FxLMS) in method for obtaining 
% reference signal. FbLMS does not employ any reference sensor. Instead, it
% uses linear predictor for generating reference signal. Thus, FbLMS is 
% suitable for reducing narrow-band noise. 
%
% Here is the system block diagram. 
%
%                 +-----------+                       +   
%    x(k) ------->|   P(z)    |--yp(k)----------------> sum --+---> e(k)
%                 +-----------+                           ^-  |
%                                                         |   |
%      +-------------------------------+                  |   |
%      |                               |                  |   |
%      |              \                |                ys(k) |     
%      |          +-----------+        |  +-----------+   |   |
%      |     +--->|   C(z)    |--yw(k)-+->|   S(z)    |---+   |
%      |     |    +-----------+           +-----------+       |
%      |     |            \                                   |
%      |     |             \-----------------\                |
%      |     |                                \               |
%      |     |    +-----------+           +-----------+       |
%      |     +--->|   Sh(z)   |--xs(k)-+->|    LMS    |<------+
%      |     |    +-----------+           +-----------+       |
%      |   xh(z)                                              |
%      |     |                            +                   |
%      |     +----------------------- sum <-------------------+
%      |                               ^+                       
%      |          +-----------+        |
%      +--------->|   Sh(z)   |--------+ 
%                 +-----------+
%
% Similar to the previous FxLMS code, I used FIR filter to model P(z), 
% C(z), S(z), and Sh(z). Imagine that the noise x(k) is propagating from 
% the source to the sensor, through the fluid medium P(z). The sensor 
% measures the arriving noise as yp(k). 
%
% To reduce noise, we generate another 'noise' yw(k) using the controller 
% C(z). We hope that it destructively interferes x(k). It means that the 
% controller has to be a model of the propagation medium P(z). Least mean 
% square algorithm is applied to adjust the controller coefficient/weight.
%
% However, there is also fluid medium S(z) that stay between the actuator 
% and sensor. We called it the secondary propagation path. So, to make the 
% solusion right, we need to compensate the adjustment process using Sh(z), 
% which is an estimate of S(z).
% 
% You can find many good information in "Active Noise Control Systems - 
% Algorithms and DSP Implementations," written by S. M. Kuo and 
% D. R. Morgan in 1996. 
%
% Let's start the code :)
%
% Developed by Agustinus Oey <oeyaugust@gmail.com>                        
% Center of Noise and Vibration Control (NoViC)                           
% Department of Mechanical Engineering                                    
% Korea Advanced Institute of Science and Technology (KAIST)              
% Daejeon, South Korea 
%--------------------------------------------------------------------------

% Set simulation duration (normalized) 
clear
T=1000; 

% We do not know P(z) and S(z) in reality. So we have to make dummy paths
Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
Sw=Pw*0.25;

% Remember that the first task is to estimate S(z). So, we can generate a
% white noise signal,
x_iden=randn(1,T);

% send it to the actuator, and measure it at the sensor position, 
y_iden=filter(Sw, 1, x_iden);

% Then, start the identification process
Shx=zeros(1,16);     % the state of Sh(z)
Shw=zeros(1,16);     % the weight of Sh(z)
e_iden=zeros(1,T);   % data buffer for the identification error

% and apply least mean square algorithm
mu=0.1;                         % learning rate
for k=1:T,                      % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error         
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
end

% Lets check the result
subplot(2,1,1)
plot([1:T], e_iden)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Identification error');
subplot(2,1,2)
stem(Sw) 
hold on 
stem(Shw, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of S(z)', 'Coefficients of Sh(z)')


% The second task is the active control itself. Again, we need to simulate 
% the actual condition. In practice, it should be an iterative process of
% 'measure', 'control', and 'adjust'; sample by sample. Now, let's generate 
% a narrow-band noise: 
X=0.1*randn(1,T)+0.9*cos(2*pi*(1/5)*[0:T-1]);

% and measure the arriving noise at the sensor position,
Yd=filter(Pw, 1, X);

% We do not have any reference signal because there is no reference sensor.
% Instead, it has to be estimated. So, lets prepare an empty buffer
Xh=zeros(size(X));
  
% Initiate the system,
Cx=zeros(1,16);       % the state of C(z)
Cw=zeros(1,16);       % the weight of C(z)
Cyx=zeros(1,16);      % the state for the estimate of control signal
Sx=zeros(size(Sw));   % the state for the secondary path
Xhx=zeros(1,16);      % the state of the filtered x(k)
e_cont=zeros(1,T);    % data buffer for the control error

% send the first sample of control signal,
k=1;
Cx=[Xh(k) Cx(1:15)];        % update the controller state
Cy=sum(Cx.*Cw);             % calculate the controller output
Sx=[Cy Sx(1:length(Sx)-1)]; % propagate to secondary path
e_cont(k)=Yd(k)-sum(Sx.*Sw);% measure the residue
    
% and start the FbLMS algorithm
mu=0.1;                              % learning rate
for k=2:T,                           % discrete time k
    Cyx=[Cy Cyx(1:15)];              % update the state for control signal
    Xh(k)=e_cont(k-1)+sum(Cyx.*Shw); % estimate reference signal
    
    Cx=[Xh(k) Cx(1:15)];             % update the controller state    
    Cy=sum(Cx.*Cw);                  % calculate the controller output	
    Sx=[Cy Sx(1:length(Sx)-1)];      % propagate to secondary path
    e_cont(k)=Yd(k)-sum(Sx.*Sw);     % measure the residue
    
    Shx=[Xh(k) Shx(1:15)];           % update the state of Sh(z)
    Xhx=[sum(Shx.*Shw) Xhx(1:15)];   % calculate the filtered x(k)
    Cw=Cw+mu*e_cont(k)*Xhx;          % adjust the controller weight
end

% Report the result
figure
subplot(2,1,1)
plot([1:T], e_cont)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise residue')
subplot(2,1,2)
plot([1:T], Yd) 
hold on 
plot([1:T], Yd-e_cont, 'r:')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise signal', 'Control signal')

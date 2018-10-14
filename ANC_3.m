M = 250;
muS = 0.1;
hNLMS = dsp.LMSFilter('Method','Normalized LMS','StepSize', muS,...
    'Length', M);
[yS,eS,Hhat] = step(hNLMS,s,dS);

n = 1:ntrS;
plot(n,dS,n,yS,n,eS);
xlabel('Number of iterations');
ylabel('Signal value');
title('Secondary Identification Using the NLMS Adaptive Filter');
legend('Desired Signal','Output Signal','Error Signal');
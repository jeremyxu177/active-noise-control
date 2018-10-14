plot(t,H,t(1:M),Hhat,t,[H(1:M)-Hhat(1:M); H(M+1:N)]);
xlabel('Time [sec]');
ylabel('Coefficient value');
title('Secondary Path Impulse Response Estimation');
legend('True','Estimated','Error');
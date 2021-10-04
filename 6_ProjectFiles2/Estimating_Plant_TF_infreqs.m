%% Creating Transfer Function using invfreqs
clear all
% Loading Sine Sweep Data
load('OpenLoop_SineSweep1.mat')
load('Omega_Array.mat')

omega_array=omega_array(1,2:end);
g_hat_array=g_hat_array(2:end,1);
theta_hat_array=theta_hat_array(2:end,1);

L=length(omega_array);

% Creating Complex Numbers from the Magnitude and Angles

for ii=1:length(omega_array)
    
   ComplexFreq_Response(1,ii) = g_hat_array(ii)*(cos(theta_hat_array(ii))+(1i)*sin(theta_hat_array(ii)));
    
end

% Creating Plant TF using invfreqs
[Num_est_TF,Den_est_TF] = invfreqs(ComplexFreq_Response,omega_array,2,4);

TF_hat_invfreqs=tf(Num_est_TF,Den_est_TF);

% Computing Frequency Response of Estimated Transfer Function
[g_hat_est_best1,theta_hat_est_best1]=bode(TF_hat_invfreqs,omega_array);

g_hat_est_best=zeros(L,1);
theta_hat_est_best=zeros(L,1);

g_hat_est_best(:,1)=g_hat_est_best1;
theta_hat_est_best(:,1)=theta_hat_est_best1;

% Plotting
bodefig = figure
ax1 = axes('position',[0.1300 0.55 0.7750 0.4])
%semilogx(w,20*log10(abs(Gjw)),'b-');
hold on;
semilogx(omega_array,20*log10(g_hat_array),'r-o');
semilogx(omega_array,20*log10(g_hat_est_best),'b--');
ylabel('gain, dB');
legend('Sine Sweep','Best est');

ax2 = axes('position',[0.1300 0.1 0.7750 0.4])
%semilogx(w,angle(Gjw)*180/pi,'b-');
hold on;
semilogx(omega_array,theta_hat_array*180/pi,'r-o');
semilogx(omega_array,theta_hat_est_best,'b--');
xlabel('\omega (rad/sec)');
ylabel('Phase, degree');
legend('Sine Sweep','Best est');

%% Storing Reults
save('Estimated_TF1.mat','TF_hat_invfreqs');
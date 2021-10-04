

%% Plotting Effect of Initial States on the Steady-State of Plant
load('EffectOfInitialStates.mat');

figure(1)
subplot(2,2,1)
plot(timeOut1,uValues1,'b.--',timeOut1,y_at_omega1,'r-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Plant-States', 'FontSize', 24)
legend('Input','Output')
title('Plant Output at 1 rad/s', 'FontSize', 24)

subplot(2,2,2)
plot(timeOut1,uValues2,'b.--',timeOut1,y_at_omega2,'r-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Plant-States', 'FontSize', 24)
legend('Input','Output')
title('Plant Output at 10 rad/s', 'FontSize', 24)

subplot(2,2,3)
plot(timeOut1,uValues3,'b.--',timeOut1,y_at_omega3,'r-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Plant-States', 'FontSize', 24)
legend('Input','Output')
title('Plant Output at 100 rad/s', 'FontSize', 24)

subplot(2,2,4)
plot(timeOut1,uValues4,'b.--',timeOut1,y_at_omega4,'r-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Plant-States', 'FontSize', 24)
legend('Input','Output')
title('Plant Output at 500 rad/s', 'FontSize', 24)

%% Plotting Nominal Frequency Response of the Plant
load('OpenLoop_SineSweep1.mat');
load('Omega_Array.mat')


    %semilogx(w,20*log10(abs(Gjw)),'b-');
    figure(2)
    subplot(2,1,1)
    semilogx(omega_array,20*log10(g_hat_array),'r-o');
    ylabel('gain, dB','FontSize',24);
    
    title('Nominal Frequency Response of Plant - Sine Sweep','FontSize',24)
    
    subplot(2,1,2)
    semilogx(omega_array,theta_hat_array*180/pi,'r-o');
    xlabel('\omega (rad/sec)','FontSize',24);
    ylabel('Phase, degree','FontSize',24);
    

%% Bode Plot of Estimated Transfer Function and Sine Sweep
load('Estimated_TF.mat');
load('Estimated_TF1.mat');
load('Omega_Array.mat');
load('OpenLoop_SineSweep1.mat');

%omega_array=[1 2 3 4 5 6 7 8 9 10 20 30 40 50 60 70  80 90 100]
L=length(omega_array);

[Num_est,Den_est]=tfdata(TF_hat_best);

% Correction for Bias
BiasCorrection_Factor=1.335;%1.335; From Open Loop Experiment

Num_est=BiasCorrection_Factor*Num_est{1};
Den_est=Den_est{1};

TF_hat_best=tf(Num_est,Den_est);

% Computing Frequency Response of Estimated Transfer Function
[g_hat_est_best1,theta_hat_est_best1]=bode(TF_hat_best,omega_array);

g_hat_est_best=zeros(L,1);
theta_hat_est_best=zeros(L,1);

g_hat_est_best(:,1)=g_hat_est_best1;
theta_hat_est_best(:,1)=theta_hat_est_best1;

[g_hat_est_best2,theta_hat_est_best2]=bode(TF_hat_invfreqs,omega_array);

g_hat_est_best1=zeros(L,1);
theta_hat_est_best1=zeros(L,1);

g_hat_est_best1(:,1)=g_hat_est_best2;
theta_hat_est_best1(:,1)=theta_hat_est_best2;

% Plotting
figure(3)
subplot(2,1,1)
semilogx(omega_array,20*log10(g_hat_array),'r-o');
hold on;
semilogx(omega_array,20*log10(g_hat_est_best),'b--');
semilogx(omega_array,20*log10(g_hat_est_best1),'g--');
ylabel('gain, dB','FontSize',24);
legend('Sine Sweep','Estimated TF - Normal', 'Estimated TF - infreqs');
title('Sine Sweep vs Estimated Plant Bode Plot','FontSize',24)
hold off

subplot(2,1,2)
semilogx(omega_array,theta_hat_array*180/pi,'r-o');
hold on;
semilogx(omega_array,theta_hat_est_best,'b--');
semilogx(omega_array,theta_hat_est_best1,'g--');
xlabel('\omega (rad/sec)','FontSize',24);
ylabel('Phase, degree','FontSize',24);
legend('Sine Sweep','Estimated TF - Normal', 'Estimated TF - infreqs');
hold off
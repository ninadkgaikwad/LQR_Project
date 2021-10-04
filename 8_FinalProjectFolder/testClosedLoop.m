%% LQR - Feedback Regulation on Estimated TF

clear all
clc
close all

%% Loading Estimated TF

% load('Estimated_TF.mat');


%% Getting Numerator and Denominator of the Estimated TF

% [Num_est,Den_est]=tfdata(TF_hat_best);

% Correction for Bias
% BiasCorrection_Factor=1.335;%1.335; From Open Loop Experiment
% 
% Num_est=BiasCorrection_Factor*Num_est{1};
% Den_est=Den_est{1};
% 
% Num_est=Num_est{1};
% Den_est=Den_est{1};

Num_est=[150.5 903 135400];
Den_est=[1 5.4 6503 4320 640000];

%% Creating State-Space Model from Estimated TF

[A_est B_est C_est D_est]=tf2ss(Num_est,Den_est); % Getting the A,B,C,D Matrices

Estimated_SS_System=ss(A_est,B_est,C_est,D_est);

%% Checking the Stability, Controllability and Observability

EigenValues_A=eig(A_est);

ConMatrix=[B_est A_est*B_est A_est^(2)*B_est A_est^(3)*B_est];
Rank_ConMatrix=rank(ConMatrix);

ObsMatrix=vertcat(C_est,C_est*A_est,C_est*A_est^(2),C_est*A_est^(3));
Rank_ObsMatrix=rank(ObsMatrix);

%% Single Experiment

SingleExperiment=0;

if (SingleExperiment==1)

    Ts = 1/300;
    freqSingleExp = 100; %rad/time unit
    amplitudeSineInput = 1;
    frequencySineInput = 10; %rad/s

    tfinal=100;

    timeU = [0:Ts:tfinal]';

    % uValues = amplitudeSineInput*sin(frequencySineInput*timeU);

    uValues = ones(length(timeU),1);

    u=zeros(length(timeU),1);
    x0=[0 0 0 0]';
    [y_est t x_est]=lsim(Estimated_SS_System,uValues,timeU,x0);
    figure(1)
    plot(t,x_est(:,1))
    hold on
    plot(t,x_est(:,2))
    plot(t,x_est(:,3))
    plot(t,x_est(:,4))
    legend('x1','x2','x3','x4')
    hold off

    % Running Test Bed Plant

    u = [];
    u.time = timeU;
    u.signals.values = uValues;
    u.signals.dimensions = 1;

    %tstart = tic;
    % ----
    Out = sim('openLoopTestBed','StopTime',num2str(tfinal));%weird,
    % ----
    %timeTaken = toc(tstart);

    % plots
    % timeOut = Out.y.time;
    % yValues = Out.y.signals.values;
    Sim_y=Out.get('y');
    Sim_signals=Sim_y.signals;
    timeOut1 = Sim_y.time;
    y_Plant = Sim_signals.values;   

    time = [0:Ts:30]';

    figure(2)
    plot(timeOut1,uValues,'b--');
    hold on
    plot(timeOut1,y_Plant,'r--');
    plot(timeOut1,y_est,'k');
    legend('Input','Plant Output','Estimated Plant Model Output')
    hold off

end

%% Simulation Time Setup

tfinal=30;
Fs=500;
Ts=1/Fs;
t=0:Ts:tfinal;

%% Finding Feedback Gain (K) using LQR

D_Transform=[1 1 1 1]; % State Transformation Matrix
Q_Scalar=400000; % Weight Matrix for States
Q=D_Transform'*Q_Scalar*D_Transform; % Transformed Weight MAtrix for the Transformed States
%Q=10^(0)*eye(4);
%Q=10^(0)*diag([10000,10,1,1]);
R=500; % Weight Matrix for Input U

[K,S,E]=lqr(Estimated_SS_System,Q,R,0);

k1=K(1);
k2=K(2);
k3=K(3);
k4=K(4);

ClosedLoop_ConPoles_est=eig(A_est-B_est*K)
ClosedLoop_ConPoles_max=max(abs(real(ClosedLoop_ConPoles_est)));

%% Computing Observer Poles (Luenberger Gain (L))
ObserverPole_Factor=[5 6.5 5.5 6];
%Observer_Poles_est=ObserverPole_Factor.*[real(ClosedLoop_ConPoles_est(1)),real(ClosedLoop_ConPoles_est(2)),real(ClosedLoop_ConPoles_est(3)),real(ClosedLoop_ConPoles_est(4))];
Observer_Poles_est=-ObserverPole_Factor*ClosedLoop_ConPoles_max

K_Dual=place(A_est',C_est',Observer_Poles_est);
L=K_Dual';

ClosedLoop_ObsPoles_est=eig(A_est-L*C_est);

L1=L(1);
L2=L(2);
L3=L(3);
L4=L(4);

K
L

%% Luenberger Observer Matrices

A_Obs_est=A_est-L*C_est;
B_Obs_est=[B_est-L*D_est L];
C_Obs_est=eye(4);
D_Obs_est=zeros(4,2);

%% Computing Stability Margins

[N_p,D_p]=ss2tf(A_est,B_est,C_est,D_est);

P_s=tf(N_p,D_p); % Plant TF

A_Con=A_est-B_est*K-L*C_est
B_Con=L;
C_Con=K;
D_Con=0;

[N_c,D_c]=ss2tf(A_Con,B_Con,C_Con,D_Con);

C_s=tf(N_c,D_c); % Controller TF

L_s=series(C_s,P_s); % Loop TF

nyquist(L_s)
[Gm,Pm,Wcg,Wcp] = margin(L_s)

%% Computing X_Star, U_Star

R_set1=0; % Given
R_set2=1; % Given

U_star1=0; % Linear Homogenous Equation Soultion

% Finding U_star2
Combined_Sys_Matrix_AB=[A_est B_est];
Combined_Sys_Matrix_CD=[C_est D_est];

Combined_Sys_Matrix=vertcat(Combined_Sys_Matrix_AB, Combined_Sys_Matrix_CD);

SteadyState_Solution=Combined_Sys_Matrix\[zeros(4,1);R_set2];

U_star2=SteadyState_Solution(5,1);


%% Generating Reference Signal

%ref_type = 'step_zero21_at5'; % -  0 up to 5 seconds, then 1
ref_type = 'step_zero21_at5_back20_at15';% -  0 at all times except between 5 and 15

t1 = [0:0.1:30]';
R_set_vector = F_ref_at_t(t1,ref_type);

%% Generating U_star_Vector

for ii=1:length(t1)
    
    if(R_set_vector(ii)==0)        
        U_star_vector(ii,1)=U_star1;        
    else       
        U_star_vector(ii,1)=U_star2;        
    end
    
end

%% Creating R_set and U_star inputs to the Simulink Simulation

R_set=[];
R_set.time=t1;
R_set.signals.values=R_set_vector;

U_star=[]
U_star.time=t1;
U_star.signals.values=U_star_vector;

%% Estimated State Space Plant - Feedback Regulation

X0_est=[0 0 0 0]';
%X0_est=randn(4,1);
X0_Obs=[0 0 0 0]';

simout_est=sim('closedLoop_Design','StopTime',num2str(tfinal))

% Gathering Simulation Data
time_est=simout_est.tout;

x1_est_hat=simout_est.xout.signals(1).values(:,1);
x2_est_hat=simout_est.xout.signals(1).values(:,2);
x3_est_hat=simout_est.xout.signals(1).values(:,3);
x4_est_hat=simout_est.xout.signals(1).values(:,4);

x1_est=simout_est.xout.signals(2).values(:,1);
x2_est=simout_est.xout.signals(2).values(:,2);
x3_est=simout_est.xout.signals(2).values(:,3);
x4_est=simout_est.xout.signals(2).values(:,4);

y_est=simout_est.y_est;
u_est=simout_est.u_est;
Ref_est=simout_est.Ref_est;

%% State Space Plant - Feedback Regulation

simout_Real=sim('closedLoop_Production','StopTime',num2str(tfinal))

% Gathering Simulation Data
time_Real=simout_Real.tout;

x1_Real_hat=simout_Real.xout.signals(1).values(:,1);
x2_Real_hat=simout_Real.xout.signals(1).values(:,2);
x3_Real_hat=simout_Real.xout.signals(1).values(:,3);
x4_Real_hat=simout_Real.xout.signals(1).values(:,4);

y_Real=simout_Real.y_Real;
u_Real=simout_Real.u_Real;
Ref_Real=simout_Real.Ref_Real;

%% Plotting Feedback Regulation

% figure(3)
% plot(time_est,x1_est,'r-',time_est,x2_est,'b-',time_est,x3_est,'g-',time_est,x4_est,'k-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Estimated Plant-States', 'FontSize', 24)
% legend('X1','X2','X3','X4')
% title('Esitmated Model - Feedback Regulation -Actual States', 'FontSize', 24)
% 
% figure(4)
% plot(time_est,x1_est_hat,'r-',time_est,x2_est_hat,'b-',time_est,x3_est_hat,'g-',time_est,x4_est_hat,'k-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Estimated Plant-States', 'FontSize', 24)
% legend('X1','X2','X3','X4', 'FontSize', 24)
% title('Esitmated Model - Feedback Regulation - Observed  States', 'FontSize', 24)

% 
% figure(5)
% plot(time_Real,x1_Real_hat,'r-',time_Real,x2_Real_hat,'b-',time_Real,x3_Real_hat,'g-',time_Real,x4_Real_hat,'k-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Real Plant-States', 'FontSize', 24)
% legend('X1','X2','X3','X4', 'FontSize', 24)
% title('Real Model - Feedback Regulation - Observed  States', 'FontSize', 24)




% figure(6)
% plot(time_Real,x1_Real_hat,'r-',time_Real,x2_Real_hat,'r--',time_Real,x3_Real_hat,'r:',time_Real,x4_Real_hat,'r.-')
% hold on
% plot(time_est,x1_est_hat,'b-',time_est,x2_est_hat,'b--',time_est,x3_est_hat,'b:',time_est,x4_est_hat,'.-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X1','X2','X3','X4')
% legend('X1-real','X2-real','X3-real','X4-real','X1-est','X2-est','X3-est','X4-est')
% title('Real Model vs Estimated Model - Feedback Regulation - Observed  States', 'FontSize', 24)
% hold off
% 
% figure(7)
% plot(time_Real,y_Real,'r-',time_est,y_est,'b-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-Outputs', 'FontSize', 24)
% legend('Y-real','Y-est')
% title('Real Model vs Estimated Model - Feedback Regulation - Outputs', 'FontSize', 24)
% 
% figure(8)
% plot(time_est,y_est,'b-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-Outputs', 'FontSize', 24)
% title('Estimated Model - Feedback Regulation - Outputs', 'FontSize', 24)
% 
% 
% figure(9)
% plot(time_Real,y_Real,'b-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-Outputs', 'FontSize', 24)
% title('Real Model - Feedback Regulation - Outputs', 'FontSize', 24)
% 
% 
% figure(10)
% plot(time_Real,u_Real,'r-',time_est,u_est,'b-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-Controller Command', 'FontSize', 24)
% legend('u-real','u-est')
% title('Real Model vs Estimated Model - Feedback Regulation - Controller Command', 'FontSize', 24)
% 
% figure(11)
% plot(time_est,u_est,'b-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-Controller Command', 'FontSize', 24)
% title(' Estimated Model - Feedback Regulation - Controller Command', 'FontSize', 24)
% 
% figure(12)
% plot(time_Real,u_Real,'b-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-Controller Command', 'FontSize', 24)
% title('Real Model - Feedback Regulation - Controller Command', 'FontSize', 24)

% figure(13)
% plot(time_est,x1_est,'r-',time_est,x2_est,'r--',time_est,x3_est,'r:',time_est,x4_est,'r.-')
% hold on
% plot(time_est,x1_est_hat,'b-',time_est,x2_est_hat,'b--',time_est,x3_est_hat,'b:',time_est,x4_est_hat,'.-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X1','X2','X3','X4')
% legend('X1','X2-l','X3','X4','X1-hat','X2-hat','X3-hat','X4-hat')
% title('Estimated Model vs Estimated Model Observer - Feedback Regulation -   States', 'FontSize', 24)
% hold off

% figure(14)
% plot(time_est,x1_est,'r-',time_est,x1_est_hat,'b--')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X1','X1-hat')
% title('Estimated Model vs Estimated Model Observer - X1 - Feedback Regulation -   States', 'FontSize', 24)
% 
% figure(15)
% plot(time_est,x2_est,'r-',time_est,x2_est_hat,'b--')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X2','X2-hat')
% title('Estimated Model vs Estimated Model Observer - X2 - Feedback Regulation -   States', 'FontSize', 24)
% 
% figure(16)
% plot(time_est,x3_est,'r-',time_est,x3_est_hat,'b--')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X3','X3-hat')
% title('Estimated Model vs Estimated Model Observer - X3 - Feedback Regulation -   States', 'FontSize', 24)
% 
% figure(17)
% plot(time_est,x4_est,'r-',time_est,x4_est_hat,'b--')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X4','X4-hat')
% title('Estimated Model vs Estimated Model Observer - X4 - Feedback Regulation -   States', 'FontSize', 24)

% plots for report
figure()
subplot(2,1,1)
plot(time_est,y_est,'r-',time_est,Ref_est,'b-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Plant Output / Reference', 'FontSize', 24)
legend('Y-Estimated Plant','Reference')
title('Closed Loop Set-Point Tracking - Estimated Plant Model','FontSize',24)
subplot(2,1,2)
plot(time_est,u_est,'b-')
hold on
plot(time_est,5*ones(length(time_est),1),'r--')
plot(time_est,-4*ones(length(time_est),1),'r-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Control Command', 'FontSize', 24)
legend('Control Command','Upper Limit','Lower Limit')
hold off

figure()
subplot(2,1,1)
plot(time_Real,y_Real,'r-',time_Real,Ref_Real,'b-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Plant Output / Reference', 'FontSize', 24)
legend('Y-Actual Plant','Reference')
title('Closed Loop Set-Point Tracking - Actual Plant','FontSize',24)
subplot(2,1,2)
plot(time_Real,u_Real,'b-')
hold on
plot(time_Real,5*ones(length(time_Real),1),'r--')
plot(time_Real,-4*ones(length(time_Real),1),'r-')
xlabel('Time (s)', 'FontSize', 24)
ylabel('Control Command', 'FontSize', 24)
legend('Control Command','Upper Limit','Lower Limit')
hold off

% figure(13)
% plot(time_est,x1_est,'r-',time_est,x2_est,'r--',time_est,x3_est,'r:',time_est,x4_est,'r.-')
% hold on
% plot(time_est,x1_est_hat,'b-',time_est,x2_est_hat,'b--',time_est,x3_est_hat,'b:',time_est,x4_est_hat,'.-')
% xlabel('Time (s)', 'FontSize', 24)
% ylabel('Plant-States', 'FontSize', 24)
% legend('X1','X2','X3','X4')
% legend('X1','X2-l','X3','X4','X1-hat','X2-hat','X3-hat','X4-hat')
% title('Estimated Model vs Estimated Model Observer - Feedback Regulation -   States', 'FontSize', 24)
% hold off


K
L
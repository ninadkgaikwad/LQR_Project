% an example of how to conduct open loop tests on
% the "virtual plant" provided as a Simulink model.
% Prabir Barooah, for Spring 2019, EML 5311
%
% 1. Put all these files in a folder, and then 'cd'
% to that folder from the matlab command prompt before 
% you try anything. 
% 2. The simulink model uses a Level-2 S function, 
% which contains a mathematical model of the plant
% that is used for simulation. The S-function details are hidden
% in the file 'virtualPlant.p'
% Simulink sometimes gives strange errors unless you are working
% in the same folder in which the S function (the .p file
% and all the Simulink models) reside. Hence point 1
% 3. I have designed the Simulink model so that the sampling period
% can be changed by you (Ts), and the 'from workspace' and 'to workspace'
% variables are saved at that sampling period. When you start making
% changes to the files, especially in testing closed loop controller,
% you may add additional 'to/from workspace' blocks, but make sure
% they follow the same philosophy. Always verify from the data
% that the sampling period has not been messed up
% 4. Related to 3: I have found that it is best to
% use "structure with time" for the 'to/from workspace' blocks
% to ensure every signal gets sampled at the specified rate.
% That is what I have used here, and I strongly suggest you do the same
% when you test your closed loop design.


clear all


modelName = 'openLoopTestBed';

Ts=0.1; %sampling period, in seconds (rather, time units)
tfinal = 100;
timeInput = [0:Ts:tfinal]';


%%---
% construct the input structure
%uValues = 1*sin(50*timeInput);

uValues = ones(length(timeInput),1);

u = [];
u.time = timeInput;
u.signals.values = uValues;
u.signals.dimensions = 1;

%%- run simulation
Out = sim(modelName,'StopTime',num2str(tfinal));
%%--

%% Running the best TF

load('Estimated_TF.mat');

%% Getting Numerator and Denominator of the Estimated TF

[Num_est,Den_est]=tfdata(TF_hat_best);

% Correction for Bias
BiasCorrection_Factor1=1;%1.335; From Open Loop Experiment
BiasCorrection_Factor2=1.335;%1.335

Num_est1=BiasCorrection_Factor1*Num_est{1};
Num_est2=BiasCorrection_Factor2*Num_est{1};
Den_est=Den_est{1};

%% Creating State-Space Model from Estimated TF

[A_est B_est C_est D_est]=tf2ss(Num_est1,Den_est); % Getting the A,B,C,D Matrices

Estimated_SS_System=ss(A_est,B_est,C_est,D_est);

Y_estSS = lsim(Estimated_SS_System,uValues,timeInput);

[A_est B_est C_est D_est]=tf2ss(Num_est2,Den_est); % Getting the A,B,C,D Matrices

Estimated_SS_System1=ss(A_est,B_est,C_est,D_est);

Y_estSS1 = lsim(Estimated_SS_System1,uValues,timeInput);

%% plot the results

timeOut = Out.y.time;
yValues = Out.y.signals.values;

figure(1)
plot(timeInput,uValues,'b--');
hold on
plot(timeOut,yValues,'r-');
plot(timeOut,Y_estSS,'g-');
plot(timeOut,Y_estSS1,'k-');

xlabel('Time (s)','FontSize',24);
ylabel('Input/Output Magnitude','FontSize',24);
title('Open Loop Test Actual vs Estimated Plant Models','FontSize',24);
legend('u','y-plant','y- approximate k1','y- accurate k1');



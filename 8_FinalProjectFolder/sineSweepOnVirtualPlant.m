%% an example of sine sweep identification of frequency response of a KNOWN plant, so that the answer can be verified
%  Prabir Barooah, Spring 2019
% I have coded the algorithm and verified that it works, meaning, it estimates
% the frequency response of the plant quite accurately even with large amounts of 
% sensor noise, as long as the parameters that are up to the designer (transiet time to ignore, 
% number of periods to average over etc.) are chosen appropriately. You only need to pick 
% appropriate values of those parametrs and hit the run button.
%% The default sine sweep parameters  I have included below 
%% ARE DELIBERATELY POORLY CHOSEN.  YOU WILL HAVE TO DO A FEW SINGLE-SINE EXPERIMENTS 
%% (USE THE FIRST SEGMENT OF THE SCRIPT) -- AND THINK -- TO FIGURE OUT APPROPRIATE VALUES

clear all
clc
%%------

%% user inputs (common):

modelName = 'openLoopTestBed';

conductSingleExp=0;
conductSineSweepExp=1;

A_u=1;

%% more user inputs
if conductSingleExp==1
    
    Ts = 1/300;
    freqSingleExp = 100; %rad/time unit
    amplitudeSineInput = 1;
    frequencySineInput = 100; %rad/s
    tfinal = 50;
    
end

%% user inputs (for sine sweep only)
if conductSineSweepExp==1
    %--------------------------
     %omega_array = [100]; %it needs to be a row or column vecor of frequencies in 
     
     % Generating Range of Frequencies
     LowPower=1; % Lowest Power of 10^(a)
     HighPower=2; % Lowest Power of 10^(b)
     %n1=100; % Number of smaple frequenceis per power of 10
     %n2=(abs(a1)+abs(b1))*n1; % Total number of sample frequencies 
     
     %omega_array = logspace(a1,b1,n2);
     
     [omega_array] = omega_array_Creator(LowPower,HighPower);
     
     AdditionalOmega=[0.1 0.5 12 15 18 125 150 175 250 500];
     
     omega_array=horzcat(omega_array,AdditionalOmega) 
     
     omega_array=sort(omega_array); % Ascending Order
     
    % "radians/time unit"
    %--------------------------
    
    %--------------------------
    decay_time = [30]; %in "time units" % Extra Initial time given for Output to settle
    %--------------------------
    
    %--------------------------
    num_cycles2average_nom = [25]; %needs to be a positive integer % Number of Steady State Cycles after Deacy time to be used for Sine-Sweep
    %--------------------------
    
    
     
    NyqistFactor=2.5; % For practical purposes it should be 5-10 times
    Fs=NyqistFactor*max(omega_array);%sampling period in "samples/time unit" % Nyquist Criteria Sampling Frequency >= 2*max(omega_array)
    % choose it large enough to give you enough samples in one period of the sinusoid
    % (depends on the highest frequency you use, of course), but not so
    % high that the simulation takes foreever.
    Fs = [1000];
    Ts = 1/Fs;

end


%% PRELIMINARY INVESTIGATION OF TIME RESPONSE AT SOME ARBITRARY FREQUENCY
if conductSingleExp ==1
    
    timeU = [0:Ts:tfinal]';
    uValues = amplitudeSineInput*sin(frequencySineInput*timeU);

    u = [];
    u.time = timeU;
    u.signals.values = uValues;
    u.signals.dimensions = 1;

    tstart = tic;
    %% ----
    Out = sim(modelName,'StopTime',num2str(tfinal));%weird,
    % ----
    timeTaken = toc(tstart);


    %% plots
    % timeOut = Out.y.time;
    % yValues = Out.y.signals.values;
    Sim_y=Out.get('y');
    Sim_signals=Sim_y.signals;
    timeOut1 = Sim_y.time;
    y_at_omega = Sim_signals.values;    
    

    time = [0:Ts:100]';


    
    figure(1)
    plot(timeOut1,uValues,'b.--',timeOut1,y_at_omega,'r');
    legend('u','y');
    
end

if conductSineSweepExp==1
    g_hat_array = nan*ones(length(omega_array),1);
    theta_hat_array = nan*ones(length(omega_array),1);
    
    time = [0:Ts:100]';
    
    for omega_index = 1:length(omega_array)
        
        
        
        omega = omega_array(omega_index);
        
        disp(['Starting with freq = ',num2str(omega),' rad/sec']);
        
        num_cycles2average = num_cycles2average_nom+5*ceil(omega);%this increases 
        % the value of N for higher frequencies. 
        
        if (omega==0)
            
            tfinal = decay_time+10;        
            timeU = [0:Ts:tfinal]';
            uValues = A_u*sin(omega*timeU);            
            
        else
            
            tfinal = decay_time+(2*pi/omega)*num_cycles2average;        
            timeU = [0:Ts:tfinal]';
            uValues = A_u*sin(omega*timeU);            
            
        end
        

        
       
        %% do experiment to colect data
        u = [];
        u.time = timeU;
        u.signals.values = uValues;
        u.signals.dimensions = 1;

        tstart = tic;
        %% ----
        Out = sim(modelName,'StopTime',num2str(tfinal));%weird,
        % ----
        timeTaken = toc(tstart);


        % plots
        % timeOut = Out.y.time;
        % yValues = Out.y.signals.values;
        Sim_y=Out.get('y');
        Sim_signals=Sim_y.signals;
        timeOut1 = Sim_y.time;
        y_at_omega = Sim_signals.values;        
        
        
        %%
        
        %cut the transient period out, so that the data is consistent with the
        %theory of sine in sine out
        inds2average = [ceil(decay_time/Ts):1:length(timeU)]'; %%%%%%
        N = length(inds2average); %number of samples to average over
        
        cosine_vector = cos(omega*timeU);  %%%%%%25 cylces
        sine_vector = sin(omega*timeU); %%%%%%
        ZcN = y_at_omega(inds2average)'*cosine_vector(inds2average);
        ZsN = y_at_omega(inds2average)'*sine_vector(inds2average);
        g_hat_omega = 2/A_u/N*sqrt(ZcN^2+ZsN^2); %gain est
        theta_hat_omega = atan2(ZcN,ZsN); %phase est, in rad
        
        %save estimates
        g_hat_array(omega_index) = g_hat_omega;
        theta_hat_array(omega_index) = theta_hat_omega;
        
        disp(['done with freq = ',num2str(omega),' rad/sec']);
    end
    
    %% compute the true magnitude and phase at a large number of frequencies:
%     w = logspace(-2,3,1000);
%     [Gjw] = freqresp(Plant,w);
%     Gjw = Gjw(:);

% [omega_array,Index1]=sort(omega_array);
% 
% for jj=1:length(Index1)
%     
%     g_hat_array1(jj,1)=g_hat_array(Index1(jj));
%     theta_hat_array1(jj,1)=theta_hat_array(Index1(jj));
%     
% end
% 
% g_hat_array=g_hat_array1;
% theta_hat_array=theta_hat_array1;
    
    %% superimpose the estimate and the true frequency response on the same Bode plot
    figure()
    subplot(2,1,1)
    semilogx(omega_array,20*log10(g_hat_array),'r-o');
    ylabel('gain, dB','FontSize',24);
    
    title('Nominal Frequency Response of Plant - Sine Sweep','FontSize',24)
    
    subplot(2,1,2)
    semilogx(omega_array,theta_hat_array*180/pi,'r-o');
    xlabel('\omega (rad/sec)','FontSize',24);
    ylabel('Phase, degree','FontSize',24);
    
    % Saving Sine Sweep Results
    save('OpenLoop_SineSweep3.mat','g_hat_array','theta_hat_array','omega_array')
    
end

%% Estimation of Plant - Infreqs() Method

% Loading Sine Sweep Data
load('OpenLoop_SineSweep1.mat')
load('Omega_Array.mat')

L=length(omega_array);

% Creating Complex Numbers from the Magnitude and Angles

for ii=1:length(omega_array)
    
   ComplexFreq_Response(1,ii) = g_hat_array(ii)*(cos(theta_hat_array(ii))+(1i)*sin(theta_hat_array(ii)));
    
end

% Creating Plant TF using invfreqs
[Num_est_TF,Den_est_TF] = invfreqs(ComplexFreq_Response,omega_array,2,4);

TF_hat_invfreqs=tf(Num_est_TF,Den_est_TF);

% Computing Frequency Response of Estimated Transfer Function
[g_hat_est_best12,theta_hat_est_best12]=bode(TF_hat_invfreqs,omega_array);

g_hat_est_best1=zeros(L,1);
theta_hat_est_best1=zeros(L,1);

g_hat_est_best1(:,1)=g_hat_est_best12;
theta_hat_est_best1(:,1)=theta_hat_est_best12;

% Storing Reults
save('Estimated_TF1.mat','TF_hat_invfreqs','g_hat_est_best1','theta_hat_est_best1');

%% Estimation of Plant Model - Heuristic Method

% Loading Sine Sweep Experiment Data0
load('OpenLoop_SineSweep1.mat')
load('Omega_Array.mat')

% Creating Damping Ratio Vector
Zeta_Vector=0:0.01:0.1; % 0 >= Zeta <=1

% Creating Counter
Counter=0;

% Creating Damping Ratio Matrix
Zeta_Matrix=zeros(1,3) % Initializing Damping Ratio Matrix

for ii=1:length(Zeta_Vector)
   
    for jj=1:length(Zeta_Vector)
        
       for kk=1:length(Zeta_Vector) 
           
           Counter=Counter+1; % Incrementing Counter
           
           Zeta_Matrix(Counter,:)=[Zeta_Vector(ii) Zeta_Vector(jj) Zeta_Vector(kk) ];
           
       end
        
    end
    
end

[R, C]=size(Zeta_Matrix); % Getting Size of Damping R Matrix

% Gain and Corener Frequencies from Sinesweep

G_w0=g_hat_array(2); % Converting Gain at W=0 rads/s to normal Gain

W_nz=30;
W_np1=10;
W_np2=80;
k1=(G_w0*W_np1^(2)*W_np2^(2))/(W_nz^(2));

L=length(omega_array);

g_hat_est_matrix=zeros(L,1);
theta_hat_est_matrix=zeros(L,1);

% Finding the Best Fit
for ii=1:R % For all rows of Damping Ratio Matrix
    
    
    ii
    
    % Getting Zeta1, Zeta2, Zeta3
    Zeta1=Zeta_Matrix(ii,1);
    Zeta2=Zeta_Matrix(ii,2);
    Zeta3=Zeta_Matrix(ii,3);
    
    % Creating Transfer Function
    TF1=tf([0,k1],[0,1]);
    TF2=tf([1,2*Zeta1*W_nz,W_nz^(2)],[0,1]);
    TF3=tf([0,1],[1,2*Zeta2*W_np1,W_np1^(2)]);
    TF4=tf([0,1],[1,2*Zeta3*W_np2,W_np2^(2)]);
    TF_1=series(TF1,TF2);
    TF_2=series(TF3,TF4);
    %Num=[k1,k1*2*Zeta1*W_nz,k1*W_nz^(2)];
    %Den=[1,2*Zeta3*W_np2+2*Zeta2*W_np1,W_np1^(2)+W_np2^(2)+4*Zeta2*Zeta3*W_np2*W_np1,2*Zeta2*W_np1*W_np2^(2)+2*Zeta3*W_np2*W_np1^(2),W_np1^(2)*W_np2^(2)];
    
    %TF_hat=tf(Num,Den);
    TF_hat=series(TF_1,TF_2);
    
    % Computing Frequency Response of Estimated Transfer Function
    [mag,phase]=bode(TF_hat,omega_array);
    
    % Storing Frequency Response of Estimated Transfer Function
    g_hat_est_matrix(:,ii)=mag;
    theta_hat_est_matrix(:,ii)=phase;
    
    % Computing RMSE Error for Frequency Response of the Esitmated Transfer
    % Function
    g_hat_rmse(ii)=sqrt((sum((g_hat_est_matrix(:,ii)-g_hat_array).^(2)))/L);
    theta_hat_rmse(ii)=sqrt((sum((theta_hat_est_matrix(:,ii)-theta_hat_array).^(2)))/L);
    
    %total_rmse(ii)=g_hat_rmse(ii)+theta_hat_rmse(ii);
    total_rmse(ii)=g_hat_rmse(ii);
    
       
end

% Getting the Best Fit from Total RMSE Values
[min_total_rmse,min_rmse_index]=min(total_rmse);

% Getting Best Damping Ratio values
Zeta1_best=Zeta_Matrix(min_rmse_index,1);
Zeta2_best=Zeta_Matrix(min_rmse_index,2);
Zeta3_best=Zeta_Matrix(min_rmse_index,3);

% Computing Best Estimated TF
% Num=[k1,2*Zeta1_best*W_nz,W_nz^(2)];
% Den=[1,2*Zeta3_best*W_np2+2*Zeta2_best*W_np1,W_np1^(2)+W_np2^(2)+4*Zeta2_best*Zeta3_best*W_np2*W_np1,2*Zeta2_best*W_np1*W_np2^(2)+2*Zeta3_best*W_np2*W_np1^(2),W_np1^(2)*W_np2^(2)]
% 
% TF_hat_best=tf(Num,Den);

    TF1=tf([0,k1],[0,1]);
    TF2=tf([1,2*Zeta1_best*W_nz,W_nz^(2)],[0,1]);
    TF3=tf([0,1],[1,2*Zeta2_best*W_np1,W_np1^(2)]);
    TF4=tf([0,1],[1,2*Zeta3_best*W_np2,W_np2^(2)]);
    TF_1=series(TF1,TF2);
    TF_2=series(TF3,TF4);modelName = 'openLoopTestBed';

    %Num=[k1,k1*2*Zeta1*W_nz,k1*W_nz^(2)];
    %Den=[1,2*Zeta3*W_np2+2*Zeta2*W_np1,W_np1^(2)+W_np2^(2)+4*Zeta2*Zeta3*W_np2*W_np1,2*Zeta2*W_np1*W_np2^(2)+2*Zeta3*W_np2*W_np1^(2),W_np1^(2)*W_np2^(2)];
    
    %TF_hat=tf(Num,Den);
    TF_hat_best=series(TF_1,TF_2);

% Computing Frequency Response of Estimated Transfer Function
[g_hat_est_best11,theta_hat_est_best11]=bode(TF_hat_best,omega_array);

g_hat_est_best=zeros(L,1);
theta_hat_est_best=zeros(L,1);

g_hat_est_best(:,1)=g_hat_est_best11;
theta_hat_est_best(:,1)=theta_hat_est_best11;

% Storing Reults
save('Estimated_TF.mat','k1','TF_hat_best','Zeta1_best','Zeta2_best','Zeta3_best');

% Plots

figure()
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

%% Open Loop Test

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

%load('Estimated_TF.mat');

%% Getting Numerator and Denominator of the Estimated TF

[Num_est,Den_est]=tfdata(TF_hat_best);

[Num_est1,Den_est1]=tfdata(TF_hat_invfreqs);

% Correction for Bias
% BiasCorrection_Factor1=1;%1.335; From Open Loop Experiment
% BiasCorrection_Factor2=1;%1.335;%1.335
% 
% Num_est1=BiasCorrection_Factor1*Num_est{1};
% Num_est2=BiasCorrection_Factor2*Num_est{1};
% Den_est=Den_est{1};

Num_est=Num_est{1};
Den_est=Den_est{1};

Num_est1=Num_est1{1};
Den_est1=Den_est1{1};
% Creating State-Space Model from Estimated TF

[A_est B_est C_est D_est]=tf2ss(Num_est1,Den_est1); % Getting the A,B,C,D Matrices

Estimated_SS_System=ss(A_est,B_est,C_est,D_est);

Y_estSS = lsim(Estimated_SS_System,uValues,timeInput);

[A_est B_est C_est D_est]=tf2ss(Num_est,Den_est); % Getting the A,B,C,D Matrices

Estimated_SS_System1=ss(A_est,B_est,C_est,D_est);

Y_estSS1 = lsim(Estimated_SS_System1,uValues,timeInput);

% plot the results

timeOut = Out.y.time;
yValues = Out.y.signals.values;

figure()
plot(timeInput,uValues,'b--');
hold on
plot(timeOut,yValues,'r-');
plot(timeOut,Y_estSS,'g-');
plot(timeOut,Y_estSS1,'k-');

xlabel('Time (s)','FontSize',24);
ylabel('Input/Output Magnitude','FontSize',24);
title('Open Loop Test Actual vs Estimated Plant Models','FontSize',24);
legend('u','y-plant','y- invfreqs()','y- Heuristic Method');
%legend('u','y-plant','y- approximate k1','y- accurate k1');


%% Function for creating Omega's
function [omega_array] = omega_array_Creator(LowPower,HighPower)
% Creating Logarithmically spaced values 

Vector_Power=LowPower:HighPower; % Vector of powers of 10
Required_Multiples=[1 2 3 4 5 6 7 8 9 10];

for ii=1:length(Vector_Power)-1 % For each Power of 10
    
    if (ii==1)    
     Current_OmegaVector=10^(Vector_Power(ii))*Required_Multiples;
     
     omega_array=Current_OmegaVector;     
    else    
        Current_OmegaVector=10^(Vector_Power(ii))*Required_Multiples(2:end);

        omega_array=horzcat(omega_array,Current_OmegaVector);    
    end    
    
end

end

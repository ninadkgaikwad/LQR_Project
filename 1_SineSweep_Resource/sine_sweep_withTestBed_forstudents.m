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


%%------

%% user inputs (common):

modelName = 'testbed_R2016a';

conductSingleExp=1;
conductSineSweepExp=1;

A_u=1;

%% more user inputs
if conductSingleExp==1
    
    Ts = 1/300;
    freqSingleExp = 100; %rad/time unit
    amplitudeSineInput = 1;
    frequencySineInput = 10; %rad/s
    tfinal = 30;
    
end

%% user inputs (for sine sweep only)
if conductSineSweepExp==1
    %--------------------------
     %omega_array = [100]; %it needs to be a row or column vecor of frequencies in 
     
     % Generating Range of Frequencies
     LowPower=0; % Lowest Power of 10^(a)
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
    decay_time = [25]; %in "time units" % Extra Initial time given for Output to settle
    %--------------------------
    
    %--------------------------
    num_cycles2average_nom = [10]; %needs to be a positive integer % Number of Steady State Cycles fter Deacy time to be used for Sine-Sweep
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
    

    time = [0:Ts:30]';


    
    figure
    plot(timeOut1,uValues,'b.--',timeOut1,y_at_omega,'r');
    
end

if conductSineSweepExp==1
    g_hat_array = nan*ones(length(omega_array),1);
    theta_hat_array = nan*ones(length(omega_array),1);
    
    for omega_index = 1:length(omega_array)
        
        
        
        omega = omega_array(omega_index);
        
        disp(['Starting with freq = ',num2str(omega),' rad/sec']);
        
        num_cycles2average = num_cycles2average_nom+5*ceil(omega);%this increases 
        % the value of N for higher frequencies. 
        
        tfinal = decay_time+(2*pi/omega)*num_cycles2average;        
        timeU = [0:Ts:tfinal]';
        uValues = A_u*sin(omega*timeU);
        
       
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
        inds2average = [ceil(decay_time/Ts):1:length(time)]';
        N = length(inds2average); %number of samples to average over
        
        cosine_vector = cos(omega*time);
        sine_vector = sin(omega*time);
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
    bodefig = figure
    ax1 = axes('position',[0.1300 0.55 0.7750 0.4])
    %semilogx(w,20*log10(abs(Gjw)),'b-');
    hold on;
    semilogx(omega_array,20*log10(g_hat_array),'r-o');
    ylabel('gain, dB');
    legend('true','est');
    
    ax2 = axes('position',[0.1300 0.1 0.7750 0.4])
    %semilogx(w,angle(Gjw)*180/pi,'b-');
    hold on;
    semilogx(omega_array,theta_hat_array*180/pi,'r-o');
    xlabel('\omega (rad/sec)');
    ylabel('Phase, degree');
    legend('true','est');
    
end



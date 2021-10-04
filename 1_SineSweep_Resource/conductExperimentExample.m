%% an example provided to specify an input to a plant in the simulink model named 
% 'testbed', run the model, collect the output and both input and output.

clear all


%% specify properties of the simulink model
modelName = 'testbed_R2016a';
% specify 'Ts' - sampling time for both input and output of the simulink
% block
Ts=0.01; %sampling period, in seconds (time unit)

%% ---
% construct the input structure

amplitudeSineInput = 1;
frequencySineInput = 10; %rad/s
tfinal = 30;

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
timeOut = Sim_y.time;
yValues = Sim_signals.values;

figure
plot(timeU,uValues,'b*--',timeOut,yValues,'r*-');
xlabel('time');ylabel('u and y');
legend('u','y');
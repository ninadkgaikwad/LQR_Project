% Example showing how to generate the two types of reference signals
% you need to test your desing on.
% Prabir Barooah, Spring 2019

clear all

ref_type = 'step_zero21_at5'; % -  0 up to 5 seconds, then 1
%ref_type = 'step_zero21_at5_back20_at15';% -  0 at all times except between 5 and 15
% seconds, when it is 1

t = [0:0.1:30]';
r = F_ref_at_t(t,ref_type);
figure
plot(t,r,'b.-')
xlabel('time');ylabel('reference');
axis([0 30 -1 2]);

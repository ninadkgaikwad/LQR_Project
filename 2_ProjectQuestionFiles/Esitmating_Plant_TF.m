%% Finding Best Damping Ratio Combination

% Loading Sine Sweep Experiment Data
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
    TF_2=series(TF3,TF4);
    %Num=[k1,k1*2*Zeta1*W_nz,k1*W_nz^(2)];
    %Den=[1,2*Zeta3*W_np2+2*Zeta2*W_np1,W_np1^(2)+W_np2^(2)+4*Zeta2*Zeta3*W_np2*W_np1,2*Zeta2*W_np1*W_np2^(2)+2*Zeta3*W_np2*W_np1^(2),W_np1^(2)*W_np2^(2)];
    
    %TF_hat=tf(Num,Den);
    TF_hat_best=series(TF_1,TF_2);

% Computing Frequency Response of Estimated Transfer Function
[g_hat_est_best1,theta_hat_est_best1]=bode(TF_hat_best,omega_array);

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
legend('Sine Sweep','Best Estimated Model');

ax2 = axes('position',[0.1300 0.1 0.7750 0.4])
%semilogx(w,angle(Gjw)*180/pi,'b-');
hold on;
semilogx(omega_array,theta_hat_array*180/pi,'r-o');
semilogx(omega_array,theta_hat_est_best,'b--');
xlabel('\omega (rad/sec)');
ylabel('Phase, degree');
legend('Sine Sweep','Best Estimated Model');

%% Storing Reults
save('Estimated_TF_original.mat','k1','TF_hat_best','Zeta1_best','Zeta2_best','Zeta3_best');












































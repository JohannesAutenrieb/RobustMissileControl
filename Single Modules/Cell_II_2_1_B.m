%%%========================================================================
%% Cell for Question 2.1 c): Weighting filter selection

%% Step : Inner Loop Gain
%% Defining Variables
Z_alpha=-1231.914;
M_q=0;
Z_delta=-107.676;
A_alpha=-1429.131;
A_delta=-114.59;
V=947.684;
g=9.81;
omega_a=150;
zeta_a=0.7;
r_M_alpha=57.813;
r_M_delta=32.716;

% Uncertain parameters
M_alpha=ureal('M_alpha',-299.26,'Percentage',[-r_M_alpha, +r_M_alpha]);
M_delta=ureal('M_delta',-130.866,'Percentage',[-r_M_delta, +r_M_delta]);

%% Defining the Actuator System Matrices of Actuators
A_ac= [ 0 1 ; (-omega_a^2) (-2*zeta_a*omega_a)];
B_ac= [ (0); (omega_a^2)];
C_ac= [ (1); 0]';
D_ac= 0;

% Creating State Space System Model
Gss_ac = ss(A_ac,B_ac,C_ac,D_ac,'StateName',{'\delta_q','\delta_q_dot'},'InputName',{'\delta_q_c'},'OutputName',{'\delta_q'});

% Defining the Airframe System Matrices
A_af= [ (Z_alpha/V) 1 ; (M_alpha) (M_q)];
B_af= [ (Z_delta/V); (M_delta)];
C_af= [ (A_alpha/g) 0 ;
        0 1 ]';
D_af= [A_delta/g 0]';

% Creating State Space System Model of Airframe
Gss_af = ss(A_af,B_af,C_af,D_af,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});


%% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

%% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

%% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

%% Reference model computation:
% Transfer Function Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);

% printing relevant step information
step_information = stepinfo(T_r_m)











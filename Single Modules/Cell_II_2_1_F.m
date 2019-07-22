%%%========================================================================
%% Cell for Question 2.1 b): Reference model computation

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

%% Step : Outter Loop Gain

% %Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

% Combining the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

%% Reference model computation:

% computing the non minimum phase zero of 
inner_zeros=zero(T_inner(1));
z_n_m_p=inner_zeros(1);

%% Parameter Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);
T_r_m.InputName = {'a_z_c'}; 
T_r_m.OutputName = {'a_z_t'};

%% Weighting Filter Selection

%% Sensitivity function Analysis
s_t=1-T_r_m;
w_s=1/s_t;

% get gain respones of system
[mag_s,phase,wout] = bode(s_t);
% [Gm,Pm,Wcg,Wcp] = margin(w_s);

% compute further needed parameter
omega_i_s=6.81; %from bode plot at gain -3 db
k_i_s=0.707;

% high and low frequency gain
lfgain_s = mag_s(1);
hfgain_s = mag_s(end);

% needed pole
omega_i_stroke_s = sqrt((hfgain_s^2-k_i_s^2)/(k_i_s^2-lfgain_s^2))*omega_i_s;

%% Complementary sensitivity function Analysis
% w_t=tf(T_r_m);
w_t=tf(Gss_ac);

% get gain respones of system
[mag_t,phase,wout] = bode(T_r_m);
% [Gm,Pm,Wcg,Wcp] = margin(w_t);

% compute further needed parameter
omega_i_t=20.8; %from bode plot at gain -3 db
k_i_t=0.707;

% high and low frequency gain
lfgain_t = mag_t(1);
hfgain_t = mag_t(end);

% needed pole
omega_i_stroke_t = sqrt((hfgain_t^2-k_i_t^2)/(k_i_t^2-lfgain_t^2))*omega_i_t;
 
% Weight definition
s = zpk('s');
W1 = inv((hfgain_s*s+omega_i_stroke_s*lfgain_s)/(s+omega_i_stroke_s));
W2 = inv((hfgain_t*s+omega_i_stroke_t*lfgain_t)/(s+omega_i_stroke_t));
W3 = [];

%sensitivty function values
S_o=(1/W1);
KS_o=(1/W2); 

% mixsyn shapes the singular values of the sensitivity function S
% [K,CL,GAM] = mixsyn(T_r_m,W1,W2,W3);

%% Synthesis block diagram

%Summing junctions
Sum_a_in = sumblk('a_z_m_d = a_z_m + a_z_d');
Sum_a_rm = sumblk('a_z_tmd = a_z_t- a_z_m_d');

%change of input names
sum_r = sumblk('r = a_z_c');
sum_r.OutputName = {'a_z_c'}; 
sum_r.InputName = {'r'};

sum_d = sumblk('d = a_z_d');
sum_d.OutputName = {'a_z_d'}; 
sum_d.InputName = {'d'};

sum_u = sumblk('u = q_s_c');
sum_u.OutputName = {'q_s_c'}; 
sum_u.InputName = {'u'};

%Gain Matrices
K_z = tunableGain('K_z',2,2);
K_z.Gain.Value = eye(2);
K_z.InputName = {'a_z_tmd','a_z_d'}; 
K_z.OutputName = {'z_1_tild','z_2_tild'};

K_v = tunableGain('K_v',2,2);
K_v.Gain.Value = eye(2);
K_v.InputName = {'a_z_m_d','a_z_c'}; 
K_v.OutputName = {'v_1','v_2'};


%% Compute orange box transfer function
P_s_tild= connect(T_outter,T_r_m,K_z,K_v,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tild','z_2_tild','v_1','v_2'});

%% Compute orange + Green box transfer function

%Weight Matrices
% Component 1
Tf_ws_1 = tf(W1);
Tf_ws_1.InputName='z_1_tild';
Tf_ws_1.OutputName='z_1';
% Component 2
Tf_ws_2 = tf(W2);
Tf_ws_2.InputName='z_2_tild';
Tf_ws_2.OutputName='z_2';

% Connceted system
P_s= connect(P_s_tild,Tf_ws_1,Tf_ws_2,{'r','d','u'},{'z_1','z_2','v_1','v_2'});

%% Create Augmented Plant for H-Infinity Synthesis
% P = augw(P_s_tild,W1,W2);
%total number of controller inputs
NMEAS=2;
% total number of controller outputs
NCON=1;

%hinfstruct
[K_F0,CL,GAM] = hinfsyn(P_s,NMEAS,NCON); 

% obtaining the decomposed control system gains
K_dr=K_F0(1);
K_cf=K_F0(2);

% minimal control orders
K_dr=tf(minreal(K_dr));
K_cf=tf(minreal(K_cf));

% obtain converted controllers
Kprime_cf=K_cf/K_dr;
Kprime_dr=K_dr;

%% Controller analysis

% order of the controller
order(Kprime_cf);
order(Kprime_dr);


% poles and zeros of the controller
figure
iopzplot(Kprime_cf);
figure
iopzplot(Kprime_dr);


% frequency behaviour of the controllers
figure
bode(Kprime_cf);
figure
bode(Kprime_dr);

%% Controller Reduction
Kprime_cf_red =reduce(Kprime_cf,3);
Kprime_dr_red =reduce(Kprime_dr,3);


%% Controller analysis

% order of the controller
order(Kprime_cf_red);
order(Kprime_dr_red);


% poles and zeros of the controller
figure
iopzplot(Kprime_cf_red);
figure
iopzplot(Kprime_dr_red);


% frequency behaviour of the controllers
figure
bode(Kprime_cf_red);
figure
bode(Kprime_dr_red);




















%%%========================================================================
%% Cell for Question 1.2 a): Uncertain inner loop state space model

%% Step : Inner Loop Gain
% Defining Variables
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

%Defining the Actuator System Matrices of Actuators
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

% %%Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.25;%-0.165;%0.00782; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

%% Open Loop System
T_open = ss(connect(Gss_af,Gss_ac,Gss_se,'\delta_q_c',{'a_z_m','q_m'}));

%% Open Loop Response Analysis

% Saving Plotsin directory
mkdir('./img/Cell_1_2_A');

% Nyquist - Uncertain
figure
nyquist(T_open(1))
hold on
nyquist(-T_open(1))
title("Open Loop Root Locus - Uncertain System");
print('./img/Cell_1_2_A/open_nyquist_unc','-dsvg');

% Nyquist - nominal
figure
nyquist(ss(T_open(1)))
hold on
nyquist(ss(-T_open(1)))
title("Open Loop Root Locus - Uncertain System");
print('./img/Cell_1_2_A/open_nyquist_nom','-dsvg');

% Root Locus - Uncertain
figure
rlocus(T_open(1))
zeta=0.707
sgrid(zeta,20000)
title("Open Loop Root Locus - Uncertain System");
print('./img/Cell_1_2_A/open_rlocus_unc','-dsvg');

% Root Locus - Nominal
figure
rlocus(ss(T_open(1)))
zeta=0.707
sgrid(zeta,20000)
title("Open Loop Root Locus - Nominal System");
print('./img/Cell_1_2_A/open_rlocus_nom','-dsvg');

% Step response of the uncertain system
figure
step(T_open)
title("Open Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/open_step_unc','-dsvg');

% Step response of the nominal system
figure
step(ss(T_open))
title("Open Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/open_step_nom','-dsvg');

%% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

%% Inner Loop Response Analysis: Computing gains for damping = 0.7 of  SISO uncertain model q_c --> a_z

%Root Locus - uncertain
figure
rlocus(T_inner(1))
zeta=0.707
sgrid(zeta,20000)
title("Inner Loop Root Locus - Uncertain System");
print('./img/Cell_1_2_A/inner_rlocus_unc','-dsvg');

%Root Locus - nominal
figure
rlocus(ss(T_inner(1)))
zeta=0.707
sgrid(zeta,20000)
title("Inner Loop Root Locus - Nominal System");
print('./img/Cell_1_2_A/inner_rlocus_nom','-dsvg');

%step response of the uncertain system
figure
step(T_inner)
title("Inner Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/inner_step_unc','-dsvg');

%step response of the nominal system
figure
step(ss(T_inner))
title("Inner Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/inner_step_nom','-dsvg');

%%%========================================================================
%% Step : Outter Loop Gain

% %Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

% Combining the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

%% Outter Loop Response Analysis
%step response of the uncertain system
figure
step(T_outter)
title("Outter Loop Step Response - Uncertain System");
print('./img/Cell_1_2_A/outter_step_unc','-dsvg');

%step response of the nominal system
figure
step(ss(T_outter))
title("Outter Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/outter_step_nom','-dsvg');

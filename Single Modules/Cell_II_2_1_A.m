%%%========================================================================
%% Cell for Question 2.1 a): Verify inner loop system

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
K_q.Gain.Value = -1*0.141; %-0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

%% open Loop System
T_open = connect(Gss_af,Gss_ac,Gss_se,'\delta_q_c',{'a_z_m','q_m'});

%% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});
 
%% Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.Gain.Value = 2.3761; 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';


%% Inner Loop Analysis: Computing gains for damping = 0.7 of  SISO uncertain model q_c --> a_z

% Saving Plotsin directory
mkdir('./img/Cell_2_1_A');

%getting Transferfunction of inner loop
inner_tf=tf(T_inner)

%ploting the poles and zeros Uncertain 
figure
iopzplot(T_inner);
title("Z-Plane Diagram - Inner Loop System");
print('./img/Cell_2_1_A/ZPlane_unc','-dsvg');

%ploting the poles and zeros Nominal
figure
iopzplot(ss(T_inner));
title("Z-Plane Diagram - Inner Loop System");
print('./img/Cell_2_1_A/ZPlane_nom','-dsvg');

%step response of the uncertain system
figure
step(T_inner)
title("Inner Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/inner_step_unc','-dsvg');

% Information of Step Respomse: settling time and overshoot
inner_step=stepinfo(T_inner);

% Time it takes for the errors between the response y(t) and the steady-state response yfinal to fall to within 2% of yfinal.
settlingTime=inner_step.SettlingTime
% Percentage overshoot, relative to yfinal).
overShot=inner_step.Overshoot

%step response of the nominal system
figure
step(ss(T_inner))
title("Inner Loop Step Response - Nominal System");
print('./img/Cell_1_2_A/inner_step_nom','-dsvg');

%% Connceting the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});








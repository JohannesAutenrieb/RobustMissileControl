%% ==================================================================================================
% Cell for Question 1.1 a): Creating the System Model for the nominal and  the uncertain System
%% ==================================================================================================

%====== Nominal Model
%Defining Variables[M,Delta] = lftdata(Gss_af)

Z_alpha=-1231.914;
M_q=0;
Z_delta=-107.676;
A_alpha=-1429.131;
A_delta=-114.59;
V=947.684;
g=9.81;
omega_a=150;
zeta_a=0.7;

%uncertain parameters
M_alpha=-299.26;
M_delta=-130.866;

%%Defining the Actuator System Matrices
A_ac_n= [ 0 1 ; (-omega_a^2) (-2*zeta_a*omega_a)];
B_ac_n= [ (0); (omega_a^2)];
C_ac_n= [ (1) 0];
D_ac_n= 0;

%Creating State Space System Model
Gss_ac_n = ss(A_ac_n,B_ac_n,C_ac_n,D_ac_n,'StateName',{'\delta_q','\delta_q_dot'},'InputName',{'\delta_q_c'},'OutputName',{'\delta_q'});


% Defining the Airframe System Matrices
A_af_n= [ (Z_alpha/V) 1 ; (M_alpha) (M_q)];
B_af_n= [ (Z_delta/V); (M_delta)];
C_af_n= [ (A_alpha/g) 0 ;
        0 1 ]';
D_af_n= [A_delta/g 0]';

%Creating State Space System Model
Gss_af_n = ss(A_af_n,B_af_n,C_af_n,D_af_n,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});


% Defining the Sesnsor System Matrices
A_se_n= [0 0; 0 0];
B_se_n= [0  0; 0 0];
C_se_n= [0 0;0 0];
D_se_n= [1 0; 0 1];

Gss_se_n = ss(A_se_n,B_se_n,C_se_n,D_se_n,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

%%Combining the Models
T_n = connect(Gss_af_n,Gss_ac_n,Gss_se_n,'\delta_q_c',{'a_z_m','q_m'});

%computing poles
p_n= pole(T_n)

%computing zeros
z_n = zero(T_n)

%computing daming of nominal model
eig(T_n);
damp(T_n);

% Creating and Saving Plots
% saving directory
mkdir('./img/Cell_1_1_A');

%step response of nominal system
figure
step(T_n)
title("Step Response - Nominal System");
print('./img/Cell_1_1_A/Step_Nom','-dsvg');

%impulse response of nominal system
figure
impulse(T_n)
title("Impulse Response - Nominal System");
print('./img/Cell_1_1_A/Impulse_Nom','-dsvg');

% Bode plot of nominal system
figure
bode(T_n)
title("Bode Diagram - Uncertain System");
print('./img/Cell_1_1_A/Bode_Un','-dsvg');

%impulse response of uncertain system
figure
nyquist(T_n)
title("Nyquist Diagram - Nominal System");
print('./img/Cell_1_1_A/Nyquist_Nom','-dsvg');


%ploting the poles and zeros
figure
iopzplot(T_n(1));
title("Z-Plane Diagram - Nominal System a_z");
print('./img/Cell_1_1_A/ZPlane_a_z_Nom','-dsvg');

%ploting the poles and zeros
figure
iopzplot(T_n(2));
title("Z-Plane Diagram - Nominal System q");
print('./img/Cell_1_1_A/ZPlane_q_Nom','-dsvg');


%%%========================================================================
%%%========================================================================

%====== Uncertain Model

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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});


%Combining the Models
T1 = connect(Gss_af,Gss_ac,Gss_se,'\delta_q_c',{'a_z_m','q_m'});

% computing daming of unvertain model
eig(T1);
damp(T1);


% Creating and Saving Plots

% step response of uncertain system
figure
step(T1)
title("Step Response - Uncertain System");
print('./img/Cell_1_1_A/Step_Un','-dsvg');

% impulse response of unvertain system
figure
impulse(T1)
title("Impulse Response - Uncertain System");
print('./img/Cell_1_1_A/Impulse_Un','-dsvg');

% Bode plot
figure
bode(T1)
title("Bode Diagram - Uncertain System");
print('./img/Cell_1_1_A/Bode_Un','-dsvg');

% nyquist plot
figure
nyquist(T1)
title("Nyquist Diagram - Uncertain System");
print('./img/Cell_1_1_A/Nyquist_Un','-dsvg');


% ploting the poles and zeros
%for output az
figure
iopzplot(T1(1));
title("Z-Plane Diagram - Uncertain System a_z");
print('./img/Cell_1_1_A/ZPlane_az_Un','-dsvg');

%for output q
figure
iopzplot(T1(2));
title("Z-Plane Diagram - Uncertain System q");
print('./img/Cell_1_1_A/ZPlane_q_Un','-dsvg');

% legend('RMSE', 'CRLB')
% title('EKF RMSE (m)')
% xlabel('Time step')
% ylabel('RMSE (m)')
% print('./Graphs/EKF','-dsvg')


%% ==================================================================================================
%Cell for Question 1.1 b): LFT model
%% ==================================================================================================

%====== Uncertain Model

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

% Defining the Airframe System Matrices
A_af= [ (Z_alpha/V) 1 ; (M_alpha) (M_q)];
B_af= [ (Z_delta/V); (M_delta)];
C_af= [ (A_alpha/g) 0 ;
        0 1 ]';
D_af= [A_delta/g 0]';

% Creating State Space System Model of Airframe
Gss_af = ss(A_af,B_af,C_af,D_af,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});

% Decomposing Uncertain Objects Decomposing Uncertain Objects
[M,Delta,BLK] = lftdata(Gss_af)

% LFT State Space System 
LFT = ss(M);

%Simplify
ssLFT=simplify(LFT,'full')

% Validation: Theoreticly Expected Matrices 
A_est= [ (Z_alpha/V) 1 ; (M_alpha.NominalValue) (M_q)];
B_est= [ 0 0 (Z_delta/V); 
         sqrt(-M_alpha.NominalValue*(r_M_alpha/100)) sqrt(-M_delta.NominalValue*(r_M_delta/100)) M_delta.NominalValue];
C_est= [ sqrt(-M_alpha.NominalValue*(r_M_alpha/100)) 0 ;
         0 0 ;
         (A_alpha/g) 0 ;
         0 1 ];
D_est= [ 0 0 0 ;
         0 0 sqrt(-M_delta.NominalValue*(r_M_delta/100));
         0 0 (A_delta/g) ;
        0 0 0 ];
    
% Comparision with compute H matrix elements (Succesful validation:  All matrix elements equals zero)
Zero_A= A_est-ssLFT.A
Zero_B= B_est-ssLFT.B
Zero_C= C_est-ssLFT.C
Zero_D= D_est-ssLFT.D

%% ==================================================================================================
% Cell for Question 1.2 a): Uncertain inner loop state space model
%% ==================================================================================================

% Step : Inner Loop Gain
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


% Defining the Sesnsor System Matrices
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


% LTI GAIN system for simulink
K_q_sim=tf(-0.165); % -0.165 for q_m found via rlocus
K_q_sim.Inputname = 'e_q'; 
K_q_sim.OutputName = '\delta_q_c';



%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% Open Loop System
T_open = ss(connect(Gss_af,Gss_ac,Gss_se,'\delta_q_c',{'a_z_m','q_m'}));

% Open Loop Response Analysis

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

% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

% Inner Loop Response Analysis: Computing gains for damping = 0.7 of  SISO uncertain model q_c --> a_z

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
% Step : Outter Loop Gain

% %Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';


% define K_sc for unitary steady state gain
K_sc_sim=tf(1/(-7.9849));  %gain found via bodeplot
K_sc_sim.Inputname = 'q_s_c'; 
K_sc_sim.OutputName = 'q_c';


% Combining the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

%transfer function of outter closed loop
G_in=tf(T_outter(1));

% Outter Loop Response Analysis
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


%% ==================================================================================================
% Cell for Question 1.2 b): Uncertainty weight computation
%% ==================================================================================================

% Step : Inner Loop Gain
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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');
 
% Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.Gain.Value = 2.3761; 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

% Connceting the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

% Decomposing Uncertain ObjectsDecomposing Uncertain Objects
[M,Delta] = lftdata(T_outter)

% Samples of uncertain state space system
parray = usample(T_outter,50);
parray_a_z = usample(T_outter(1),50);

nom_sys=ss(T_outter);

% Creating and Saving Plots
% saving directory
mkdir('./img/Cell_1_2_B');

% Nominal Bode
figure
bode(parray,'b--',nom_sys,'r',{.1,1e3}), grid
legend('Frequency response data','Nominal model','Location','NorthEast');
print('./img/Cell_1_2_B/bode_Nom_Unc','-dsvg');

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1] = ucover(parray_a_z,nom_sys(1),1,'InputMult');
[P2,InfoS2] = ucover(parray_a_z,nom_sys(1),2,'InputMult');
[P3,InfoS3] = ucover(parray_a_z,nom_sys(1),3,'InputMult');
[P4,InfoS4] = ucover(parray_a_z,nom_sys(1),4,'InputMult');

% Bode plot weigthed trans function: order 
Gp=ss(parray_a_z);
G=nom_sys(1);
relerr = (Gp-G)/G;
%Bode Magnjitude 
figure
bodemag(relerr,'b',InfoS1.W1,'r-.',InfoS2.W1,'k-.',InfoS3.W1,'m-.',InfoS4.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)First-order w','Lm(\omega)Second-order w','Lm(\omega)Third-order w','Lm(\omega)Fourth-order w','Location','SouthWest');
title("Singular Values - Multiplicative Form (Order Comparison)")
print('./img/Cell_1_2_B/bode_mag_weights_order','-dsvg');

% Bode plot weigthed trans function: samples 

% Samples of uncertain state space system
parray_a_z1 = usample(T_outter(1),10);
parray_a_z2 = usample(T_outter(1),20);
parray_a_z3 = usample(T_outter(1),30);
parray_a_z4 = usample(T_outter(1),40);

nom_sys=ss(T_outter);

% Creating and Saving Plots
% saving directory
mkdir('./img/Cell_1_2_B');

% Nominal Bode
figure
bode(parray,'b--',nom_sys,'r',{.1,1e3}), grid
legend('Frequency response data','Nominal model','Location','NorthEast');
print('./img/Cell_1_2_B/bode_Nom_Unc','-dsvg');

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1] = ucover(parray_a_z1,nom_sys(1),2,'InputMult');
[P2,InfoS2] = ucover(parray_a_z2,nom_sys(1),2,'InputMult');
[P3,InfoS3] = ucover(parray_a_z3,nom_sys(1),2,'InputMult');
[P4,InfoS4] = ucover(parray_a_z4,nom_sys(1),2,'InputMult');

% Bode plot weigthed trans function: order 
Gp=ss(parray_a_z);
G=nom_sys(1);
relerr = (Gp-G)/G;
%Bode Magnjitude 
figure
bodemag(relerr,'b',InfoS1.W1,'r-.',InfoS2.W1,'k-.',InfoS3.W1,'m-.',InfoS4.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)- 10 Samples','Lm(\omega) - 20 Samples','Lm(\omega) - 30 Samples','Lm(\omega)- 40 Samples','Location','SouthWest');
title("Singular Values - Multiplicative Form (Sample Comparison)")
print('./img/Cell_1_2_B/bode_mag_weights_sampels_multi','-dsvg');

%%+===============================
%%Additive
%%+===============================

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1A] = ucover(parray_a_z,nom_sys(1),1,'Additive');
[P2,InfoS2A] = ucover(parray_a_z,nom_sys(1),2,'Additive');
[P3,InfoS3A] = ucover(parray_a_z,nom_sys(1),3,'Additive');
[P4,InfoS4A] = ucover(parray_a_z,nom_sys(1),4,'Additive');

% Bode plot weigthed trans function: order 
Gp=ss(parray_a_z);
G=nom_sys(1);
relerr = (Gp-G)/G;

%Bode Magnjitude 
figure
bodemag(relerr,'b',InfoS1A.W1,'r-.',InfoS2A.W1,'k-.',InfoS3A.W1,'m-.',InfoS4A.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)First-order w','Lm(\omega)Second-order w','Lm(\omega)Third-order w','Lm(\omega)Fourth-order w','Location','SouthWest');
title("Singular Values - Multiplication Form (Order Comparison)")
print('./img/Cell_1_2_B/bode_mag_weights_order_addi','-dsvg');


figure
bode(InfoS1.W1);
title('Scalar Additive Uncertainty Model')
legend('First-order w','Min. uncertainty amount','Location','SouthWest');
print('./img/Cell_1_2_B/bode_weight1_addi_order','-dsvg');

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1A] = ucover(parray_a_z1,nom_sys(1),2,'Additive');
[P2,InfoS2A] = ucover(parray_a_z2,nom_sys(1),2,'Additive');
[P3,InfoS3A] = ucover(parray_a_z3,nom_sys(1),2,'Additive');
[P4,InfoS4A] = ucover(parray_a_z4,nom_sys(1),2,'Additive');

% Bode plot weigthed trans function: order 
Gp=ss(parray_a_z);
G=nom_sys(1);
relerr = (Gp-G)/G;
%Bode Magnjitude 
figure
bodemag(relerr,'b',InfoS1A.W1,'r-.',InfoS2A.W1,'k-.',InfoS3A.W1,'m-.',InfoS4A.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)- 10 Samples','Lm(\omega) - 20 Samples','Lm(\omega) - 30 Samples','Lm(\omega)- 40 Samples','Location','SouthWest');
print('./img/Cell_1_2_B/bode_mag_weights_sampels_addi','-dsvg');
title("Singular Values - Additive Form ((Sample Comparison)");
print('./img/Cell_1_2_B/bode_weight1_addi_samples','-dsvg');

%%+===============================
figure
bode(InfoS1.W1);
title('Scalar Additive Uncertainty Model')
legend('First-order w','Min. uncertainty amount','Location','SouthWest');
print('./img/Cell_1_2_B/bode_weight1_addi','-dsvg');


%% ==================================================================================================
% Cell for Question 2.1 a): Verify inner loop system
%% ==================================================================================================

% Step : Inner Loop Gain

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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -1*0.141; %-0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% open Loop System
T_open = connect(Gss_af,Gss_ac,Gss_se,'\delta_q_c',{'a_z_m','q_m'});

% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});
 
% Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.Gain.Value = 2.3761; 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';


% Inner Loop Analysis: Computing gains for damping = 0.7 of  SISO uncertain model q_c --> a_z

% Saving Plotsin directory
mkdir('./img/Cell_2_1_A');

%getting Transferfunction of inner loop
inner_tf=tf(T_inner)

%ploting the poles and zeros Uncertain 
figure
iopzplot(T_inner);
title("Z-Plane Diagram - Inner Closed Loop System (Uncertain)");
print('./img/Cell_2_1_A/ZPlane_unc','-dsvg');

%ploting the poles and zeros Nominal
figure
iopzplot(ss(T_inner));
title("Z-Plane Diagram -  Inner Closed Loop System (Nominal)");
print('./img/Cell_2_1_A/ZPlane_nom','-dsvg');

%step response of the uncertain system
figure
step(T_inner)
title("IStep Response -  Inner Closed Loop System (Uncertain)");
print('./img/Cell_2_1_A/inner_step_unc','-dsvg');

% Information of Step Respomse: settling time and overshoot
inner_step=stepinfo(T_inner);

% Time it takes for the errors between the response y(t) and the steady-state response yfinal to fall to within 2% of yfinal.
settlingTime=inner_step.SettlingTime
% Percentage overshoot, relative to yfinal).
overShot=inner_step.Overshoot

%step response of the nominal system
figure
step(ss(T_inner))
title("Step Response -  Inner Closed Loop System (Nominal)");
print('./img/Cell_1_2_A/inner_step_nom','-dsvg');

% Connceting the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});


%% ==================================================================================================
%Cell for Question 2.1 b): Reference model computation
%% ==================================================================================================

% Step : Inner Loop Gain
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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% Reference model computation:
% Transfer Function Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% non minimum phase zero of system (closed loop system)
z_n_m_p=36.6;

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);

% printing relevant step information
step_information = stepinfo(T_r_m)


%% ==================================================================================================
% Cell for Question 2.1 c): Weighting filter selection
%% ==================================================================================================


% Step : Inner Loop Gain
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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

% Reference model computation:

% computing the non minimum phase zero of 
inner_zeros=zero(T_inner(1));
z_n_m_p=inner_zeros(1);

% Parameter Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);

%Step: Weighting filter selection

% Sensitivity function Analysis
s_t=1-T_r_m;
% get gain respones of system
[mag_s,phase,wout] = bode(s_t);

% compute further needed parameter
omega_i_s=6.81; %from bode plot at gain -3 db
k_i_s=0.707;

% high and low frequency gain
lfgain_s = mag_s(1);
hfgain_s = mag_s(end);
% needed pole
omega_i_stroke_s = sqrt((hfgain_s^2-k_i_s^2)/(k_i_s^2-lfgain_s^2))*omega_i_s;

% Complementary sensitivity function Analysis
w_t=tf(T_r_m);
% w_t=tf(Gss_ac);

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
 
% weight definition
s = zpk('s');
W1 = inv((hfgain_s*s+omega_i_stroke_s*lfgain_s)/(s+omega_i_stroke_s));
W2 = inv((hfgain_t*s+omega_i_stroke_t*lfgain_t)/(s+omega_i_stroke_t));
W3 = [];

%sensitivty function values
S_o=(1/W1);
KS_o=(1/W2); 

% mixsyn shapes the singular values of the sensitivity function S
% [K,CL,GAM] = mixsyn(T_r_m,W1,W2,W3);

% Saving Plotsin directory
mkdir('./img/Cell_2_1_C');

%plot of singular system diagram
figure
sigma(s_t,'r-',T_r_m,'c-',S_o,'k-',KS_o,'g-')
legend('S_t','T_{RM}','S_o','KS_o','Location','Northwest')
title("Singular Values - Weight Filter");
print('./img/Cell_2_1_C/sigma_weights','-dsvg');

% ilustrate target transfer function
% plot S_o
figure
bodemag(S_o,'r');
title("Singular Values - S_0");
print('./img/Cell_2_1_C/sigma_weights_s_0','-dsvg');


%plot KS_o
figure
bodemag(KS_o,'r')
title("Singular Values - KS_0");
print('./img/Cell_2_1_C/sigma_weights_ks_0','-dsvg');

%% ==================================================================================================
% Cell for Question 2.1 d): Synthesis block diagram
%% ==================================================================================================
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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

% Step : Outter Loop Gain

% %Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

% Combining the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

% Reference model computation:

% computing the non minimum phase zero of 
inner_zeros=zero(T_inner(1));
z_n_m_p=inner_zeros(1);

% Parameter Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);
T_r_m.InputName = {'a_z_c'}; 
T_r_m.OutputName = {'a_z_t'};

% Weighting filter selection

% Sensitivity function Analysis
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

% Complementary sensitivity function Analysis
w_t=tf(T_r_m);
% w_t=tf(Gss_ac);

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
 
% weight definition
s = zpk('s');
W1 = inv((hfgain_s*s+omega_i_stroke_s*lfgain_s)/(s+omega_i_stroke_s));
W2 = inv((hfgain_t*s+omega_i_stroke_t*lfgain_t)/(s+omega_i_stroke_t));
W3 = [];

% mixsyn shapes the singular values of the sensitivity function S
[K,CL,GAM] = mixsyn(T_r_m,W1,W2,W3);

% Synthesis block diagram

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
K_z.InputName = {'a_z_tmd','q_s_c'}; 
K_z.OutputName = {'z_1_tild','z_2_tild'};

K_v = tunableGain('K_v',2,2);
K_v.Gain.Value = eye(2);
K_v.InputName = {'a_z_m_d','a_z_c'}; 
K_v.OutputName = {'v_1','v_2'};


% Compute orange box transfer function
P_s_tild= connect(T_outter,T_r_m,K_z,K_v,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tild','z_2_tild','v_1','v_2'});

% Transferfunction
Tf_p_s = tf(P_s_tild);

% Compute orange + Green box transfer function

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

% Give out final Transfer function
tf_tild=tf(P_s_tild);
tf_full=tf(P_s);


%% ==================================================================================================
% Cell for Question 2.1 e): Controller Synthesis
%% ==================================================================================================

% Step : Inner Loop Gain
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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

% paraemter for simulink modul
K_q_value=-0.165;

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

% Step : Outter Loop Gain

% %Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

% Combining the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

% Reference model computation:

% computing the non minimum phase zero of 
inner_zeros=zero(T_inner(1));
z_n_m_p=inner_zeros(1);

% Parameter Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);
T_r_m.InputName = {'a_z_c'}; 
T_r_m.OutputName = {'a_z_t'};

%Weighting Filter Selection

% Sensitivity function Analysis
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

% Complementary sensitivity function Analysis
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

% Synthesis block diagram

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


% Compute orange box transfer function
P_s_tild= connect(T_outter,T_r_m,K_z,K_v,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tild','z_2_tild','v_1','v_2'});

% Compute orange + Green box transfer function

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

% Create Augmented Plant for H-Infinity Synthesis


% P = augw(P_s_tild,W1,W2);
%total number of controller inputs
NMEAS=2;
% total number of controller outputs
NCON=1;

%hinfstruct
[K_F0,CL,GAM] = hinfsyn(P_s,NMEAS,NCON); 

% % further parameter
% S = (1-P_s(2,1)*K)^-1;
% R = K*S;
% T = 1-S;
% 
% figure;
% sigma(S,'m-',K_F0*S,'g-');
% legend('S_0','K*S_0','Location','Northwest');

% Plots

% saving directory
mkdir('./img/Cell_2_1_E');

figure;
bode(K_F0(2),'m-',CL,'g-');
title("Bode Diagram of K_{F0}");
print('./img/Cell_2_1_E/bode_kf0','-dsvg');

% obtaining the decomposed control system gains
K_dr=K_F0(1);
K_cf=K_F0(2);

% minimal control orders
ordDr=tf(minreal(K_dr));
ordCf=tf(minreal(K_cf));

% Weighted closed loop transfer matrix components
T_r_z1 = tf(P_s(1,1));
T_d_z1 = tf(P_s(1,2));
T_r_z2 = tf(P_s(2,1));
T_d_z2 = tf(P_s(2,2));

% Unweighted closed loop transfer matrix components
UT_r_z1 = tf(P_s_tild(1,1));
UT_d_z1 = tf(P_s_tild(1,2));
UT_r_z2 = tf(P_s_tild(2,1));
UT_d_z2 = tf(P_s_tild(2,2));

% performance level 
perf_level=hinfnorm(CL);

% plot T_wTOztilda and superimpose it with inverse of weight functions
figure()
subplot(2,2,1)
sigma(UT_r_z1)
grid on
hold on
sigma(UT_r_z1*S_o)
sigma(UT_r_z1*KS_o)
hold off
title('Input: r  Output: z_tild_1')

subplot(2,2,2)
sigma(UT_d_z1)
grid on
hold on
sigma(UT_d_z1*S_o)
sigma(UT_d_z1*KS_o)
hold off
title('Input: d  Output: z_tild_1')

subplot(2,2,3)
sigma(UT_r_z2)
grid on
hold on
sigma(UT_r_z2*S_o)
sigma(UT_r_z2*KS_o)
hold off
title('Input: r  Output: z_tild_2')

subplot(2,2,4)
sigma(UT_d_z2)
grid on
hold on
sigma(UT_d_z2*S_o)
sigma(UT_d_z2*KS_o)
hold off
title('Input: d  Output: z_tild_2')
% save diagram
print('./img/Cell_2_1_E/sigma_unweight_weight','-dsvg');

%adaption of different weight filter adaptions
% figure()
% for i=1:0.5:10
% % prior weightaning bandwidth    
% omega_prime1=sqrt((M_h1^2-k1^2)/(k1^2-M_l1^2))*i;
% W1=tf([1,omega_prime1],[M_h1,omega_prime1*M_l1]);
% 
% S_o=(1/W1);
% bode(S_o,'k')
% hold on
% % sigma(T_wTOztilda(1,1)*S_o,'k')
% % hold on
% % sigma(T_wTOztilda(1,2)*S_o,'k')
% % hold on
% end
% hold off

%% ==================================================================================================
% %% Cell for Question 2.1 f): Controller Implementation
%% ==================================================================================================

% Step : Inner Loop Gain
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

% Defining the Actuator System Matrices of Actuators
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


% Defining the Sesnsor System Matrices
A_se= [0 0; 0 0];
B_se= [0  0; 0 0];
C_se= [0 0;0 0];
D_se= [1 0; 0 1];

Gss_se = ss(A_se,B_se,C_se,D_se,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Definition of inner loop Gain Kq
K_q = tunableGain('K_q',1,1);
K_q.Gain.Value = -0.165; %0.00782
K_q.InputName = 'e_q'; 
K_q.OutputName = '\delta_q_c';

%Summing junction
Sum = sumblk('e_q = q_c - q_m');

% Inner Loop System
T_inner = connect(Gss_af,Gss_ac,Gss_se,K_q,Sum,'q_c',{'a_z_m','q_m'});

% Step : Outter Loop Gain

% %Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

%value parameter for simulink modul
K_s_c_value=1/dcgain(T_inner);

% Combining the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

% Reference model computation:

% computing the non minimum phase zero of 
inner_zeros=zero(T_inner(1));
z_n_m_p=inner_zeros(1);

% Parameter Computing
% tunning requirement of max. 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009

% tuning requirements of 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6

% definition of transfer function
T_r_m = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);
T_r_m.InputName = {'a_z_c'}; 
T_r_m.OutputName = {'a_z_t'};

% Weighting Filter Selection

% Sensitivity function Analysis
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

% Complementary sensitivity function Analysis
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
[K,CL,GAM] = mixsyn(T_r_m,W1,W2,W3);

% Synthesis block diagram

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


% Compute orange box transfer function
P_s_tild= connect(T_outter,T_r_m,K_z,K_v,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tild','z_2_tild','v_1','v_2'});

% Compute orange + Green box transfer function

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

% Create Augmented Plant for H-Infinity Synthesis
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

% obtain converted controllers (calculated by hand)
% Kprime_cf=K_cf/K_dr;
% Kprime_dr=1/(1-G_in.NominalValue*K_dr-G_in.NominalValue);

% obtain converted controllers
Kprime_cf=K_cf/K_dr;
Kprime_dr=-K_dr;

% Controller analysis

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

% Controller Reduction
Kprime_cf_red =reduce(Kprime_cf,3);
Kprime_dr_red =reduce(Kprime_dr,3);


% Controller analysis

% saving directory
mkdir('./img/Cell_2_1_F');


% order of the controller
order(Kprime_cf_red);
order(Kprime_dr_red);


% poles and zeros of the controller
figure
iopzplot(Kprime_cf);
title("Zeros and Poles - Kprime_cf");
print('./img/Cell_2_1_F/zero_poles_K_cf','-dsvg');

figure
iopzplot(Kprime_dr);
title("Zeros and Poles - Kprime_dr");
print('./img/Cell_2_1_F/zero_poles_K_dr','-dsvg');

% frequency behaviour of the controllers
figure
bode(Kprime_cf);
title("Bode Plot - Kprime_cf");
print('./img/Cell_2_1_F/bode_K_cf','-dsvg');

figure
bode(Kprime_dr);
title("Bode Plo - Kprime_dr");
print('./img/Cell_2_1_F/bode_K_dr','-dsvg');

% sigma values
figure
sigma(Kprime_cf_red,'r-',Kprime_dr_red,'c-',Kprime_cf,'k-',Kprime_dr,'g-')
legend('Kprime_cf-reduced','Kprime_dr-reduced','Kprime_dr','Kprime_dr','Location','Northwest')
title("Simngular Values - Full and Reduced Order");
print('./img/Cell_2_1_F/sigma_red_com','-dsvg');

%% ==================================================================================================
% %% Cell for Question 2.1 G): Controller Implementation
%% ==================================================================================================
% This Task has been done in Simulink --> Main_Simulink File

% only plots are generated here from dataloging


% Used variable is a datalog array and therefore not usable anymore

% step resonse 
% figure
% plot(solution_2_1_G(:,2))
% legend('Kprime_cf-reduced','Location','Northwest')
% title("Step Response of ");
% print('./img/Cell_2_1_G/sigma_red_com','-dsvg');
% 
% % step resonse 
% figure
% plot(solution_2_1_G(:,3))
% legend('Kprime_cf-reduced','Location','Northwest')
% title("Step Response of ");
% print('./img/Cell_2_1_G/sigma_red_com','-dsvg');
% 
% % step resonse 
% figure
% plot(solution_2_1_G(:,4))
% legend('Kprime_cf-reduced','Location','Northwest')
% title("Step Response of ");
% print('./img/Cell_2_1_G/sigma_red_com','-dsvg');
% 
% % step resonse 
% figure
% plot(solution_2_1_G(:,end))
% legend('Kprime_cf-reduced','Location','Northwest')
% title("Step Response of ");
% print('./img/Cell_2_1_G/sigma_red_com','-dsvg');

%% ==================================================================================================
% Cell for Question 3 a): Basic Robustness Analysis
%% ==================================================================================================
% This Task has been done in Simulink --> Simulink Control Design
% Please Have look on the Main_Simulink File for further understanding

%% ==================================================================================================
% Cell for Question 3 b): Advanced Robustness Analysis
%% ==================================================================================================

% Open Simulink Model and load it 
sim('Question_3');
sim_mod = 'Question_3';

% Get the analysis Input and Outputs
io = getlinio(sim_mod);

% Defining operating point
op = operpoint(sim_mod);

% Linearize the model
M = linearize(sim_mod,io,op);

M.InputName={'u1','u2'};
M.OutputName={'y1','y2'};

% Mu Analysis
norm(M,Inf);

% get mu bounds
omega = logspace(-1,2,50);
M_g = frd(M,omega);
mubnds = mussv(M_g,BLK);

% saving directory
mkdir('./img/Cell_3_B');

% plot mu bounds
figure()
LinMagopt = bodeoptions;
LinMagopt.PhaseVisible = 'off'; 
LinMagopt.MagUnits = 'abs';
bodeplot(mubnds(1,1),mubnds(1,2),LinMagopt);
xlabel('Frequency (rad/sec)');
ylabel('Mu upper/lower bounds');
title('Mu plot of robust stability margins (inverted scale)');
print('./img/Cell_3_B/mu_plot','-dsvg');


% get minimums and maximums of mu bounds
[pkl,wPeakLow] = getPeakGain(mubnds(1,2));
[pku] = getPeakGain(mubnds(1,1));
SMfromMU.LowerBound = 1/pku;
SMfromMU.UpperBound = 1/pkl;
SMfromMU.CriticalFrequency = wPeakLow;

% show values
SMfromMU;






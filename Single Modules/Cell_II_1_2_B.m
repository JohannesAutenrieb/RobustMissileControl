%%%========================================================================
%% Cell for Question 1.2 b): Uncertain inner loop state space model

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
 
%% Definition of Gain for outter Loop
K_s_c = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(T_inner);
K_s_c.Gain.Value = steady_state_gain(1); 
K_s_c.Gain.Value = 2.3761; 
K_s_c.InputName = 'q_s_c';
K_s_c.OutputName = 'q_c';

%% Connceting the Models
T_outter = connect(Gss_af,Gss_ac,Gss_se,K_q,K_s_c,Sum,'q_s_c',{'a_z_m','q_m'});

%% Decomposing Uncertain ObjectsDecomposing Uncertain Objects
[M,Delta] = lftdata(T_outter)

%% Samples of uncertain state space system
parray = usample(T_outter,50);
parray_a_z = usample(T_outter(1),50);

nom_sys=ss(T_outter);

%% Creating and Saving Plots
% saving directory
mkdir('./img/Cell_1_2_B');

%% Nominal Bode
figure
bode(parray,'b--',nom_sys,'r',{.1,1e3}), grid
legend('Frequency response data','Nominal model','Location','NorthEast');
print('./img/Cell_1_2_B/bode_Nom_Unc','-dsvg');

%% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1] = ucover(parray_a_z,nom_sys(1),1,'InputMult');
[P2,InfoS2] = ucover(parray_a_z,nom_sys(1),2,'InputMult');
[P3,InfoS3] = ucover(parray_a_z,nom_sys(1),3,'InputMult');
[P4,InfoS4] = ucover(parray_a_z,nom_sys(1),4,'InputMult');

%% Bode plot weigthed trans function: order 
Gp=ss(parray_a_z);
G=nom_sys(1);
relerr = (Gp-G)/G;
%Bode Magnjitude 
figure
bodemag(relerr,'b',InfoS1.W1,'r-.',InfoS2.W1,'k-.',InfoS3.W1,'m-.',InfoS4.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)First-order w','Lm(\omega)Second-order w','Lm(\omega)Third-order w','Lm(\omega)Fourth-order w','Location','SouthWest');
title("Singular Values")
print('./img/Cell_1_2_B/bode_mag_weights','-dsvg');

%% Bode plot weigthed trans function: samples 

%% Samples of uncertain state space system
parray_a_z1 = usample(T_outter(1),10);
parray_a_z2 = usample(T_outter(1),20);
parray_a_z3 = usample(T_outter(1),30);
parray_a_z4 = usample(T_outter(1),40);

nom_sys=ss(T_outter);

%% Creating and Saving Plots
% saving directory
mkdir('./img/Cell_1_2_B');

%% Nominal Bode
figure
bode(parray,'b--',nom_sys,'r',{.1,1e3}), grid
legend('Frequency response data','Nominal model','Location','NorthEast');
print('./img/Cell_1_2_B/bode_Nom_Unc','-dsvg');

%% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1] = ucover(parray_a_z1,nom_sys(1),2,'InputMult');
[P2,InfoS2] = ucover(parray_a_z2,nom_sys(1),2,'InputMult');
[P3,InfoS3] = ucover(parray_a_z3,nom_sys(1),2,'InputMult');
[P4,InfoS4] = ucover(parray_a_z4,nom_sys(1),2,'InputMult');

%% Bode plot weigthed trans function: order 
Gp=ss(parray_a_z);
G=nom_sys(1);
relerr = (Gp-G)/G;
%Bode Magnjitude 
figure
bodemag(relerr,'b',InfoS1.W1,'r-.',InfoS2.W1,'k-.',InfoS3.W1,'m-.',InfoS4.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)- 10 Samples','Lm(\omega) - 20 Samples','Lm(\omega) - 30 Samples','Lm(\omega)- 40 Samples','Location','SouthWest');
title("Singular Values")
print('./img/Cell_1_2_B/bode_mag_weights_sampels_multi','-dsvg');


%%+===============================
figure
bode(InfoS1.W1);
title('Scalar Additive Uncertainty Model')
legend('First-order w','Min. uncertainty amount','Location','SouthWest');
print('./img/Cell_1_2_B/bode_weights','-dsvg');


%% ==================================================================================================
% %% Cell for Question 1.1 a): Creating the System Model for the nominal and  the uncertain System
%% ==================================================================================================

%%====== Nominal Model
%Defining Variables
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


%% Defining the Airframe System Matrices
A_af_n= [ (Z_alpha/V) 1 ; (M_alpha) (M_q)];
B_af_n= [ (Z_delta/V); (M_delta)];
C_af_n= [ (A_alpha/g) 0 ;
        0 1 ]';
D_af_n= [A_delta/g 0]';

%Creating State Space System Model
Gss_af_n = ss(A_af_n,B_af_n,C_af_n,D_af_n,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});


%% Defining the Sesnsor System Matrices
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

%% Creating and Saving Plots
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


%% Defining the Sesnsor System Matrices
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


%% Creating and Saving Plots

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





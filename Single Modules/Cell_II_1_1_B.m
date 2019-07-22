%%%========================================================================
% %% Cell for Question 1.1 b): LFT model

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

%% Decomposing Uncertain Objects Decomposing Uncertain Objects
[M,Delta] = lftdata(Gss_af)

% LFT State Space System 
LFT = ss(M);

%Simplify
ssLFT=simplify(LFT,'full')

%% Validation: Theoreticly Expected Matrices 
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

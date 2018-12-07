clear; clc
% Link masses
mb1=0.12; mr1=0.04; mb2=0.084; mr2=0.057; mb3=0.127;
% Link lengths
b1=0.033; r1=0.07; b2=0.184; r2=0.152; b3=0.324;
% Other parameters
g=9.81; T=0.01;
J1= mr1*r1^2+mb1*b1^2+mb2*b2^2+(mr2+mb3)*b2^2;
J2= mr2*r2^2+mb3*b3^2;
J3= (mr2*r2+mb3*b3)*b2;

% System matrix parameters
a23= J3^2*g/(J1*J2-J3^2)/b2;
a43= J1*J3*g/(J1*J2-J3^2)/b2;
b21= J2/(J1*J2-J3^2);
b41= J3/(J1*J2-J3^2);

% Continuous State-space model
Ac=[0 1 0 0; 0 0 -a23 0; 0 0 0 1; 0 0 a43 0];
Bc=[0; b21; 0; -b41];
Cc=[1 0 0 0; 0 0 1 0];
Dc=[0; 0];

% Eigenvalues of Ac
eig(Ac)

% Number of unobservable states
CO=ctrb(Ac,Bc);
unco = length(Ac)-rank(CO)

% Number of unobservable states
OB=obsv(Ac,Cc);
unob = length(Ac)-rank(OB)

Pcld_s=[-5+j*5; -5-j*5; -15; -20];
Pestd_s=[-50; -55; -60; -65];

K_s=place(Ac,Bc,Pcld_s)
L_s=place(Ac',Cc',Pestd_s)'

% Calculate the continuous pendulum Transfer Functions
Gp_s=tf(ss(Ac,Bc,Cc,Dc))
zpk(Gp_s) % look at the system transfer functon in zero, pole, gain form

Acf=(Ac-Bc*K_s) % Compute the state matrrix with full state feedback
Hcl_s=tf(ss(Acf,Bc,Cc,Dc)) % Compute the closed-loop transfer functions with full state feedback
Kdc_s=evalfr(Hcl_s(1),0) % Calculate the DC Gain used to scale the reference input

% Calculate the Discrete State-Space mod
SYSD=c2d(ss(Ac,Bc,Cc,Dc),T,'zoh')

COd=ctrb(SYSD.A,SYSD.B)
uncod = length(SYSD.A)-rank(COd)

OBd=obsv(SYSD.A,SYSD.C)
unobd = length(SYSD.A)-rank(OBd)

Pcld_z=[0.95+j*0.05; 0.95-j*0.05; 0.5; 0.45];

K_z=acker(SYSD.A,SYSD.B,Pcld_z)
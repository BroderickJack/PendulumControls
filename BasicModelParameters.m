% Load the parameters for the inverted pendulum model

%System Parameter Values
%Masses[kg]:
mb1=0.12;
mr1=0.04;
mb2=0.084;
mr2=0.057;
mb3=0.127;

%Lengths[m]:
b1=0.033;
r1=0.07;
b2=0.184;
r2=0.152;
b3=0.324;

%Inertia
J1 = mr1*r1^2 + mb1*b1^2 + mb2*b2^2 + (mr2+mb3)*b2^2;
J2 = mr2*r2^2+mb3*b3^2;
J3 = (mr2*r2+mb3*b3)*b2;

g=9.81; %gravity

a23=(J3^2*g)/(b2*(J1*J2-J3^2));
a43=(J1*J3*g)/(b2*(J1*J2-J3^2));
b21=J2/(J1*J2-J3^2);
b41=J3/(J1*J2-J3^2);

Ac=[0 1 0 0;
    0 0 -a23 0;
    0 0 0 1;
    0 0 a43 0];
Bc=transpose([0 b21 0 -b41]);
Cc=[1 0 0 0;
    0 0 1 0];
Dc=[0; 
    0];


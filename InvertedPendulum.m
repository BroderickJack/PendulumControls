%% Jack Broderick
%  Connor Feathers
%  Frank Carag
% EE451 Inverted Pendulum Project Code
clear; clc; close all

% This model is good for theta +/15degrees (+/- 0.2618rad)

% Define the model parameters
T_STOP = 10;
MODEL_NAME = 'InvertedPendulum_REV1'; % Discretzed with a zero-order hold

Ts = 0.01; % [sec] - Sample period

% Define the controller parameters
% Controller 1
z1 = [-0.1229, -0.02181];
p1 = [-284.8, 0];
k1 = [-5.2947];

% Controller 2
z2 = [-9.249];
p2 = [-50];
k2 = [-5.1415];

% Disturbance
A_dist = 0;
f_dist = 10; % [hz];

% Theta 1 Command
theta1_cmd_A = pi/4; % [rad]
theta1_cmd_f = 0.1;   % [hz]

% Theta 2 Command
theta2_cmd_A = 0;  % [rad]
theta2_cmd_f = 10; % [hz]


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

%% Run the model
sim(MODEL_NAME);
% Plot the outputs
figure();
plot(t, control1); hold on;
plot(t, control2);
plot(t, intoModel);
title('Control Signals');
legend('Control1', 'Control2', 'Into Model');
xlabel('Time [sec]'); ylabel("Control Effort"); 
grid on;

% figure();
% plot(t, disturbance); 
% title("Disturbance");
% xlabel("Time [sec]"); ylabel("Disturbance");
% grid on;

% figure(); hold on;
% plot(t, theta1);
% plot(t, theta2);
% title("Output");
% xlabel("Time [s]"); ylabel("\theta [rad]");
% legend('\theta1','\theta2');
% grid on;

figure(); 
subplot(211);
plot(ts, theta1);
title("Theta 1");
grid on;
ylabel('\theta1 [rad]');
subplot(212);
plot(ts, theta2);
title('Theta 2');
ylabel("\theta2 [rad]");
xlabel("Time [sec]");
grid on;

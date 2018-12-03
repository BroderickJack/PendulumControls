%% Jack Broderick
%  Connor Feathers
%  Frank Carag
% EE451 Inverted Pendulum Project Code

% This used model InvertedPendulum_REV2 for computing the step response
clear; clc; close all

% This model is good for theta +/15degrees (+/- 0.2618rad)

% Define the model parameters
T_STOP = 10;
MODEL_NAME = 'InvertedPendulum_REV2'; % Discretzed with a zero-order hold

% Ts = 0.0005; % [sec] - Sample period
Ts = 0.0017; % [sec] - Sample period of the DAQ unit

GAIN = -1.2; % [A/V] - The gain of the power supply

%% Model Gains
n = 9; % Gear ratio of the motor
kt = 60*10^-3; % [Nm/A] - Torque constant of the motor
Gv = -1.2;     % [A/V] - Gain of the power supply

modelGainV = (1/n)*(1/kt)*(1/Gv); % [V/torque]
modelGainA = (1/n)*(1/kt);        % [A/torque] 

%% Define the input signal
% desiredTheta1.time = (0:Ts:T_STOP)';
% t1 = zeros(size(desiredTheta1.time));
% t1(end/2:end) = pi/4;
% desiredTheta1.signals.values = t1';
% desiredThtea1.signals.dimensions = [1];
desiredTime = 0:Ts:T_STOP;
desiredTheta = zeros(size(desiredTime));
desiredTheta(end/2:end) = pi/4;
desiredTheta1 = [desiredTime', desiredTheta'];

figure();
plot(desiredTime, desiredTheta);
xlabel('Time [sec]');
ylabel("Desired Theta 1 [rad]");
title('Desired {\theta}_{1}');
grid on;
%% Define the Controller

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

% Plot the voltages
figure();
plot(t, control1*GAIN); hold on;
plot(t, control2*GAIN);
plot(t, intoModel*GAIN);
title('Control Signals');
legend('Control1', 'Control2', 'Into Model');
xlabel('Time [sec]'); ylabel("Control Effort [A]"); 
grid on;

% Plot the derivative of the control effort
figure();
dAdt = diff(intoModel*GAIN);
plot(t(2:end),dAdt);
title('Derivative of Control Effort [A/sec]')
xlabel("Time [sec]");
ylabel('[A/sec]');
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


%% Figure out what the current command to the motor is
currentCommand = intoModel .* modelGainA;
figure();
plot(t, currentCommand);
title('Current Command');
xlabel('Time [sec]'); ylabel("Current [A]");
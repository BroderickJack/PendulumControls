%% Jack Broderick
%  Connor Feathers
%  Frank Carag
% EE451 Inverted Pendulum Project Code
clear; clc; close all

% This model is good for theta +/15degrees (+/- 0.2618rad)

% Define the model parameters
T_STOP = 10;
MODEL_NAME = 'InvertedPendulum_REV1'; % Discretzed with a zero-order hold

% Ts = 0.0005; % [sec] - Sample period
Ts = 0.0017; % [sec] - Sample period of the DAQ unit

GAIN = -1.2; % [A/V] - The gain of the power supply

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

% EE451 Final Project
% December 5, 2018
clear; clc; close all;

%% Design Requirements
Wn = 20;                % [rad/sec]
gamma = 0.707;          % [damping ratio]
Wd = Wn*sqrt(1-gamma^2);% [rad/sec]
r = exp(-1*gamma*Wn

%% Simulation Parameters
MODEL_NAME = 'InvertedPendulumPolePlacement_REV0';
T_STOP = 10;% [sec] The end time of the simulation
Ts = 0.01; % [sec] Sample period of the system (500hz)

%% Load the model parameters
BasicModelParameters

%% Calculate the pole placement gain
P_cld = [0.95+(1i*0.05); 0.95-(1i*0.05); 0.5; 0.45]; % The desired closed loop discrete time poles

% Calculate the discrete State-Space model
sys_d = c2d(ss(Ac,Bc,Cc,Dc), Ts, 'zoh'); % Calculate the discrete time state space model
Ad = sys_d.A;
Bd = sys_d.B;

% Find the gain using the 'place' function
k_z  = place(Ad, Bd, P_cld);

% Calculate steady state error 
% First we need to design a continuous domain 
Pcld_s=[-5+1i*5; -5-1i*5; -15; -20];
K_s = place(Ac, Bc, Pcld_s); % Calculate the continuous domain pole placement gain
Acf = (Ac-Bc*K_s); % The state matrix with full state feedback
Hcl_s = tf(ss(Acf, Bc, Cc, Dc)); % Closed loop transfer function
k_DC = evalfr(Hcl_s(1), 0); % Calclate the dc gain

%% Define the command to the system
desiredTime = 0:Ts:T_STOP;
desiredTheta = zeros(size(desiredTime));
desiredTheta(end/2:end) = pi/4;
desiredTheta1 = [desiredTime', desiredTheta'];
figure(); 
plot(desiredTime, desiredTheta*(180/pi));
title("Commanded \theta1");
xlabel("Time [sec]");
ylabel("Angle [deg]");
legend('\theta1 Command');

%% Simulate the model
sim(MODEL_NAME);

%% Plot the outputs from the model
figure();
plot(t, Theta1, t, Theta1Dot, t, Theta2, t, Theta2Dot);
xlabel("Time [sec]");
ylabel("Angle [rad]");
legend({'$\theta$1',"$\dot{\theta1}$", '$\theta$2', '$\dot{\theta2}$'}, 'Interpreter' ,'latex' );

figure();
subplot(211);
plot(t, Theta1*(180/pi), t, Theta2*(180/pi));
ylabel("Angle [deg]");
legend('\theta1','\theta2');
grid on;
subplot(212);
plot(t, Theta1Dot, t, Theta2Dot);
legend({"$\dot{\theta1}$", '$\dot{\theta2}$'}, 'Interpreter', 'latex');
xlabel("Time [sec]");
ylabel("[$\frac{rad}{sec}$]", 'Interpreter', 'latex');
grid on;

clear;

%% cart-pole properties
global M m l g
M = 0.5;
m = 0.5;
l = 0.3;
g = 9.81;

wheel_damping = 1e-4;
joint_damping = 1e-4;

%% cart-pole initial condition
x_0 = 0;
y_0 = 0.15;
q_0 = 10; %degree



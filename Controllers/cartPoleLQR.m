function K = cartPoleLQR
global M m l g
%% state space matrices:
A = [0 0 1 0;
    0 0 0 1;
    0 m*g/M 0 0;
    0 (m*g + M*g)/(M*l) 0 0];
B = [0;0;1/M;1/(M*l)];
C = eye(4);
D = 0;

%% build ss system:
cartpole = ss(A,B,C,D);

%% LQR:
Q = diag([100 100 10 10]); % x q dx dq
R = 1; % fx
K = lqr(cartpole, Q, R);
end
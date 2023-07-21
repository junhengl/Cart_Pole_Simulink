function Fx = NeuralNetNMPC(states)
tic
global policy
Fx = predict(policy, states');
Fx = double(Fx);
toc
end
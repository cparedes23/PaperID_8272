
clc, clear all, close all
%% .................................................................
q = rand(6,1);
qp = rand(6,1);
qpp = qp;
g = 9.8;
m7 = 10;
iexter = 0.05*eye(3);
tau = newtoneuler6(q, qp, qpp, g, m7, iexter)
acel = walkerorin6(q, qp, tau, m7, iexter)


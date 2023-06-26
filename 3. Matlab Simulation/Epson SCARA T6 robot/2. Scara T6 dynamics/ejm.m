
%%................................................................
clc, clear all, close all
%% .................................................................

q = rand(3,1);
qp = rand(3,1);
qpp = rand(3,1);
%q   = zeros(3,1);
%qp  = zeros(3,1);
%qpp = zeros(3,1);
masaext = 3;
inerciaext = 0.05*eye(3);
tau=newtoneuler3(q,qp,qpp,9.8,masaext,inerciaext)
acel = walkerorin3(q,qp,tau,masaext,inerciaext)


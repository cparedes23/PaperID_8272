% WALKERORIN3 Third method of Walker & Orin.
% QPP = WALKERORIN6(Q, QP, TAU, MASAEXT, INERCIAEXT) calculates the inverse kinematics of the 6GDL robot by returning the vector 6x1 representing the vector 6x1.
% inverse kinematics of the 6GDL robot by returning the vector 6x1 which represents the
% acceleration of each joint using the third method of Walker and
%Orin.
% Q is the 6x1 vector of joint variables. QP is the 6x1 vector that
% represents the velocity of each joint. TAU is the 6x1 vector
% representing the input torque to each joint. MASAEXT is
% is the mass of the external load. INERCIAEXT is the inertia of the external load.
%

function qpp = walkerorin6(q,qp,tau,masaext,inerciaext)
% The vector b is calculated.
b = newtoneuler6(q,qp,zeros(6,1),9.8,masaext,inerciaext);
% The moment of inertia matrix H is calculated.
H = h6(q,masaext,inerciaext);
% The acceleration vector of each joint is calculated.
qpp = inv(H)*(tau-b);

end
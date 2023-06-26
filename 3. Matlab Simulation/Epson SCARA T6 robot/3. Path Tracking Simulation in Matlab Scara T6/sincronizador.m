% SYNCHRONIZER Function that synchronizes the movements taking into account
% consideration of the initial and final conditions, characteristics of the
% articulations and nominal speeds.
% [VELO2,TMAXIMO]=SYNCHRONIZER(Q0,QF,VELO) returns the new speed
% maximum of each motor.
% Q0 initial position (joint coordinates).
% QF final position (joint coordinates).
% VELO nominal maximum speed of each actuator.
%

function [velo2,tmaximo]=sincronizador(q0,qf,velo)
%--Calculation of approximate times.
taprox = abs((qf(:,1)-q0(:,1)))./velo;
tmaximo = max(taprox);

%--New maximum speed of each motor.
velo2=(qf(:,1)-q0(:,1))/tmaximo;

return
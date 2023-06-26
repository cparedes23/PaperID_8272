%----------------------------------------------------------
%---- software test program -----
%---- drawrobot3dscara -----
%----------------------------------------------------------
clc, clear all, close all
%---------------------------------------------------------------------
%q = zeros(3,1)
q = rand(3,1)
%q = [-pi/6 pi/3 0.2];
T = directkinematicscara(q)
inversekinematicscara(T,1)
drawrobot3dscara(q)

%----------------------------------------------------------
%---- software test program -----
%---- drawrobot3d6 -----
%----------------------------------------------------------
clc, clear all, close all
%---------------------------------------------------------------------
%q = zeros(6,1)
%q = [0 pi/2 0 pi/2 pi/2 0] 
q = rand(6,1)
T = directkinematic6(q)
%inversekinematic6(T,1)
%drawrobot3d6(q)


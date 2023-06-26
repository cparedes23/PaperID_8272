% EVALVEL Function that evaluates the polynomial of the velocity function of
% according to the case and time.
%VEL=EVALVEL(T,TT,CASE,A) evaluates the acceleration polynomial.
% A 5x3 matrix containing the coefficients of the interpolated polynomials.
%T normalized time vector.
% TT acceleration, maximum speed and deceleration time segments.
% case=1: Each motor manages to reach its maximum speed.
% case=2: There is no time to reach the maximum speed of each motor.
%

function vel = evalvel(t,tt,caso,A)

for i = 1:length(t)

 if caso == 1
  if (t(i) <= 0)
     v = 0;
  elseif (t(i) > 0) & (t(i) <= tt(1))
    ti = t(i)/tt(1);
     v = (A(1,2)+2*A(1,3)*ti+3*A(1,4)*ti.^2+4*A(1,5)*ti.^3)/tt(1);
  elseif (t(i) > tt(1)) & (t(i) <= tt(2)+tt(1))
    ti = (t(i)-tt(1))/tt(2);
     v = (A(2,2)+2*A(2,3)*ti+3*A(2,4)*ti.^2)/tt(2);
  elseif (t(i) > tt(2)+tt(1)) & (t(i) <= tt(3)+tt(2)+tt(1))
    ti = (t(i)-tt(2)-tt(1))/tt(3);
     v = (A(3,2)+2*A(3,3)*(ti-1)+3*A(3,4)*(ti-1).^2+4*A(3,5)*(ti-1).^3)/tt(3);
  elseif (t(i) > tt(3)+tt(2)+tt(1))
     v = 0;
  end
 elseif caso == 2
  if (t(i) <= 0)
    v = A(1,2)/tt(1);
  elseif (t(i) > 0) & (t(i) <= tt(1))
    ti = t(i)/tt(1);
     v = (A(1,2)+2*A(1,3)*ti+3*A(1,4)*ti.^2+4*A(1,5)*ti.^3)/tt(1);
  elseif (t(i) > tt(1)) & (t(i) <= tt(2)+tt(1))
    ti = (t(i)-tt(1))/tt(2);
     v = (A(2,2)+2*A(2,3)*(ti-1)+3*A(2,4)*(ti-1).^2+4*A(2,5)*(ti-1).^3)/tt(2);
  elseif (t(i) > tt(2)+tt(1))
     v = A(2,2)/tt(2);
  end
    
 end

vel(i)=v;
end

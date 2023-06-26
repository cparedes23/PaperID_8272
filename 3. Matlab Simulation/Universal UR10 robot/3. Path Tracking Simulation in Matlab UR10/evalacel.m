% EVALACEL function that evaluates the polynomial of the acceleration function of
% according to the case and time.
%ACE=EVALACEL(T,TT,CASE,A) evaluates the acceleration polynomial.
% A 5x3 matrix containing the coefficients of the interpolated polynomials.
%T normalized time vector.
% TT acceleration, maximum speed and deceleration time segments.
% case=1: Each motor manages to reach its maximum speed.
% case=2: There is no time to reach the maximum speed of each motor.
%

function ace = evalacel(t,tt,caso,A)

for i = 1:length(t)

if caso == 1
  if (t(i) <= 0)
     a = 0;
  elseif (t(i)>0)&(t(i)<=tt(1))
    ti = t(i)/tt(1);
     a = (2*A(1,3)+6*A(1,4)*ti+12*A(1,5)*ti^2)/tt(1)^2;
  elseif (t(i)>tt(1))&(t(i)<=tt(2)+tt(1))
    ti = (t(i)-tt(1))/tt(2);
     a = (2*A(2,3)+6*A(2,4)*ti)/tt(2)^2;
  elseif (t(i)>tt(2)+tt(1))&(t(i)<=tt(3)+tt(2)+tt(1))
    ti = (t(i)-tt(2)-tt(1))/tt(3);
     a = (2*A(3,3)+6*A(3,4)*(ti-1)+12*A(3,5)*(ti-1).^2)/tt(3)^2;
  elseif (t(i) > tt(3)+tt(2)+tt(1))
     a = 0;
  end
 elseif caso == 2
  if (t(i) <= 0)
     a = A(1,3)/tt(1)^2;
  elseif (t(i) > 0) & (t(i) <= tt(1))
    ti = t(i)/tt(1);
     a = (2*A(1,3)+6*A(1,4)*ti+12*A(1,5)*ti.^2)/tt(1)^2;
  elseif (t(i) > tt(1)) & (t(i) <= tt(2)+tt(1))
    ti = (t(i)-tt(1))/tt(2);
     a = (2*A(2,3)+6*A(2,4)*(ti-1)+12*A(2,5)*(ti-1).^2)/tt(2)^2;
  elseif (t(i) > tt(2)+tt(1))
     a = A(2,3)/tt(2)^2;
  end

end

ace(i)=a;
end
 

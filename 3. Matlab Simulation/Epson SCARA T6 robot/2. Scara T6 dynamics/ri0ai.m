% RI0AI Vector ri0ai.
%Y = RI0AI(IR0VPI, IR0WPI, IR0WI, IR0SI) calculates the vector 3x1 which 
%represents the linear acceleration of the center of mass of element i. 
% IR0VPI is the 3x1 vector of linear acceleration of the i-th joint. 
%IR0WPI is the 3x1 vector of angular acceleration of the i-th joint. 
%IR0WI is the 3x1 angular velocity vector of the i-th joint. 
%IR0SI is the 3x1 vector of the position of the center of mass of the i-th element with respect to its coordinate system. 
%with respect to its coordinate system.
%

function y = ri0ai(ir0vpi, ir0wpi, ir0wi, ir0si)
   
a = cross(ir0wi, ir0si);
b = cross(ir0wi, a);
y = cross(ir0wpi, ir0si) + b + ir0vpi;

end
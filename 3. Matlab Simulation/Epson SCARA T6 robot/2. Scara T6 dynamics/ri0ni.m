% RI0NI Vector ri0ni.
% Y = RI0NI(IR0WPI, IR0WI, IR0I_0RI) calculates the vector 3x1 representing 
% the external moment exerted on element i at the center of mass. 
%IR0WPI is the 3x1 vector of angular acceleration of the i-th joint. 
%IR0WI is the 3x1 angular velocity vector of the i-th joint. 
%IR0I_0RI is the 3x3 matrix representing the inertia of the i-th element.
%

function y = ri0ni(ir0wpi, ir0wi, ir0I_0ri)

y = ir0I_0ri*ir0wpi + cross(ir0wi, (ir0I_0ri*ir0wi));

end
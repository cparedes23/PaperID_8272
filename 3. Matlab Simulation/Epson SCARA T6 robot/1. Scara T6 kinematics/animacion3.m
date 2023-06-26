% ANIMACION3 Animation of a robot path
% ANIMACION(MAT_Q) performs the animation of the trajectory, expressed in 
% the MAT_Q matrix, of a 3 GDL robot arm. MAT_Q contains 3 rows and a 
% column for each robot layout. 
%

function animacion3(mat_q)

% Position vector (x, y, z) of the reference coordinate system
x0 = 0; y0 = 0; z0 = 0;

% The reference coordinate system is drawn. The XOR mode is set to erase 
% only the previously plotted robot. A line thickness of 2 units is used.
p = plot3(x0,y0,z0,'EraseMode','xor','LineWidth',2);

% Grids are assigned to the axis
grid;

% Axis limits are established
axis([0  1  -1  1  -1  1]);

% It keeps the current graph
hold on;

% Number of columns of the matrix
n = size(mat_q,2);

% The arrangement of the robot corresponding to each column is drawn
for i=1:n
    
    % Joint variables of the robot arm
    q1i = mat_q(1,i);
    q2i = mat_q(2,i);
    q3i = mat_q(3,i);
    
    % Denavit-Hartenberg parameters of the robot
    teta  = [q1i     q2i     0        ];
    d     = [0       0      -q3i-0.083];
    a     = [0.325   0.275   0        ];
    alfa  = [0       0       0        ];
    
    % Matrices of homogeneous transformation between consecutive coordinate 
    % systems
    A01 = denavit(teta(1), d(1), a(1), alfa(1));
    A12 = denavit(teta(2), d(2), a(2), alfa(2));
    A23 = denavit(teta(3), d(3), a(3), alfa(3));
    
    % Transformation matrix from the first to the last system to the 
    % corresponding system
    A02 = A01 * A12;
    A03 = A02 * A23;
    
    % Position vector (x, y, z) of each coordinate system
    x1 = A01(1,4); y1 = A01(2,4); z1 = A01(3,4);
    x2 = A02(1,4); y2 = A02(2,4); z2 = A02(3,4);
    x3 = A03(1,4); y3 = A03(2,4); z3 = A03(3,4);
    
    % The robot is drawn
    x = [x0 x1 x2 x3];
    y = [y0 y1 y2 y3];
    z = [z0 z1 z2 z3];
    set(p,'XData',x,'YData',y,'ZData',z);
    
    % The axes are named and a grid is added
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
    grid;
    
    % The screen is forced to update in MATLAB
    drawnow;
end

end
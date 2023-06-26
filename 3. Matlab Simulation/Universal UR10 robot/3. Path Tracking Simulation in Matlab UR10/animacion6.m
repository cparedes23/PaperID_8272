% ANIMATION6 Animation of a robot trajectory
% ANIMATION(MAT_Q) performs the animation of the trajectory, expressed
% in the MAT_Q matrix, of a 6 DOF robot arm. MAT_Q contains 6 rows
% and a column for each robot layout.
%

function animacion6(mat_q)

% Measure of the links
L1 = 0.09000;
L2 = 0.61290;
L3 = 0.57160;
L4 = 0.16389;
L5 = 0.11570;
L6 = 0.17570;

% Denavit-Hartenberg parameters of the robot
d     = [L1      0       0       L4      L5     L6  ];
a     = [0       L2      L3      0       0      0   ];
alfa  = [pi/2    0       0       pi/2   -pi/2   0   ];

% Position vector (x, y, z) of the reference coordinate system
x0 = 0; y0 = 0; z0 = 0;

% The reference coordinate system is drawn. mode is assigned
%XOR to delete only the previously drawn robot. It uses a
% line thickness of 2 units
p = plot3(x0,y0,z0,'EraseMode','xor','LineWidth',2);

% A grid is assigned to the axes
grid;

% Axis limits are set
axis([-0.5 1.5 -1.5 0 -1 1.5]);

% Keeps the current graph
hold on;

% Number of columns in the matrix
n = size(mat_q,2);

% The layout of the robot corresponding to each column is drawn
for i=1:n
    
    % Joint variables of the robot arm
    teta1 = mat_q(1,i);
    teta2 = mat_q(2,i);
    teta3 = mat_q(3,i);
    teta4 = mat_q(4,i);
    teta5 = mat_q(5,i);
    teta6 = mat_q(6,i);
    
    % Homogeneous transformation matrices between coordinate systems
    A01 = denavit(teta1, d(1), a(1), alfa(1));
    A12 = denavit(teta2, d(2), a(2), alfa(2));
    A23 = denavit(teta3, d(3), a(3), alfa(3));
    A34 = denavit(teta4, d(4), a(4), alfa(4));
    A45 = denavit(teta5, d(5), a(5), alfa(5));
    A56 = denavit(teta6, d(6), a(6), alfa(6));
    
    % Transformation matrices from the first system to the corresponding
    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    A06 = A05 * A56;
    
    % Position vector (x, y, z) of each coordinate system
    x1 = A01(1,4); y1 = A01(2,4); z1 = A01(3,4);
    x2 = A02(1,4); y2 = A02(2,4); z2 = A02(3,4);
    x3 = A03(1,4); y3 = A03(2,4); z3 = A03(3,4);
    x4 = A04(1,4); y4 = A04(2,4); z4 = A04(3,4);
    x5 = A05(1,4); y5 = A05(2,4); z5 = A05(3,4);
    x6 = A06(1,4); y6 = A06(2,4); z6 = A06(3,4);
    
    L = 0.16;
    xi = L*sin(teta1); 
    yi = -L*cos(teta1); 
    zi = d(1);
    xj = a(2)*cos(teta1)*cos(teta2)+L*sin(teta1); 
    yj = a(2)*sin(teta1)*cos(teta2)-L*cos(teta1);
    zj = a(2)*sin(teta2)+d(1);
    
    % Se dibuja el robot
    x = [x0 x1 xi xj x2 x3 x4 x5 x6];
    y = [y0 y1 yi yj y2 y3 y4 y5 y6];
    z = [z0 z1 zi zj z2 z3 z4 z5 z6];
    set(p,'XData',x,'YData',y,'ZData',z);
    
    % A grid is attached to the axes
    grid on;
    xlabel('eje x');
    ylabel('eje y');
    zlabel('eje z');
    
    % Force MATLAB to update the display
    drawnow;
end

end
% H6 Moment of inertia matrix. 
% H = H6(Q, MASAEXT, INERCIAEXT) calculates the moment of inertia matrix H 6x6 using the third method of Walker and Orin. 
% H 6x6 using the third method of Walker and Orin. Q is 
% the 6x1 vector of joint variables.  MASAEXT is the mass of the 
% external load. INERCIAEXT is the inertia of the external load.
%

function  h = h6(q, masaext, inerciaext)

% Size of the links
L1 = 0.09000;
L2 = 0.61290;
L3 = 0.57160;
L4 = 0.16389;
L5 = 0.11570;
L6 = 0.17570;

% 	Denavit-Hartenberg robot parameters
teta  = [q(1)    q(2)    q(3)    q(4)    q(5)   q(6)];
d     = [L1      0       0       L4      L5     L6  ];
a     = [0       L2      L3      0       0      0   ];
alfa  = [pi/2    0       0       pi/2   -pi/2   0   ];


% Mass of each element (Kg)
m    = [3.1350;  8.8872;  4.7466;  0.7431;  0.7431;  0.4078];

% Centroidal Inertia Matrices (Kg-m^2.)
J    = [0.0125 0  0     ; 0 0.0107  0.0005;  0       0.0005 0.0012;
        0.2785 0 -0.5435; 0 1.8853  0     ; -0.5435  0      1.6255;
        0.0187 0 -0.0767; 0 0.6860  0     ; -0.0767  0      0.6731;
        0.0013 0  0     ; 0 0.0011  0     ;  0       0      0.0009;
        0.0013 0  0     ; 0 0.0011  0     ;  0       0      0.0009;
        0.0032 0  0     ; 0 0.0034 -0.0002;  0      -0.0002 0.0012];

% The seventh inertia is that of the external load.
J(19:21,1:3) = inerciaext;
      
% Vector Z0.
z0 = [0; 0; 1];

% External Loading Conditions.
M(7)  = masaext;
cj_1j = zeros(3,1);
Ej_1j = J(19:21,1:3);

for j = 6:-1:1
	% Constant to take out the Inertia J.
	k = (j-1)*3 + 1;
   
    if j==1
        p= [ 0      ;  0.09   ;  0      ];
        s= [ 0      ; -0.01152;  0.01162];   
    end
   
    if j==2
        p= [ 0.61290;  0      ; 0      ];
        s= [-0.36185;  0      ; 0.17034];   
    end
    
    if j==3
        p= [ 0.57160;  0      ;  0      ];
        s= [-0.31467;  0      ;  0.05142];   
    end
    
    if j==4
        p= [ 0      ;  0.16389;  0      ];
        s= [ 0      ; -0.00760;  0.00974];   
    end
    
    if j==5
        p= [ 0      ; -0.11570;  0      ];
        s= [ 0      ;  0.00760;  0.00974];    
    end
    
    if j==6
        p= [ 0      ;  0      ;  0.17570];
        s= [-0.00001;  0.00477; -0.07921];   
    end
    % Transformation matrices.
	aj_1j = dh(teta(j),alfa(j));
	ajj_1 = aj_1j';
   
   % Centroid and Inertia of the previous elements.
	cjjM1 = cj_1j;
	EjjM1 = Ej_1j;
   
   % Mass of all the above elements.
	M(j)  = M(j+1) + m(j);
   
   % New centroid.
	cjj   = ((s + p)*m(j) + M(j+1)*(cjjM1 + p))/M(j);
	cj_1j = aj_1j*cjj;
   
   % Inertia transfer distance.
	p1 = (cjjM1 + p - cjj);
	d1 = dot(p1,p1)*eye(3) - (p1*p1'); 
	p2 = (s + p - cjj);
	d2 = dot(p2,p2)*eye(3) - (p2*p2');
   
   % New Inertia.
	Ej_1j = aj_1j*(EjjM1 + M(j+1)*d1 + J(k:k+2,1:3) + m(j)*d2)*ajj_1;
   
   % Force and torque of j elements up to N.
	Fj_1j = cross(z0,M(j)*cj_1j);
	Nj_1j = Ej_1j*z0;
   
   % Element j. Component H(j,j).
	fi_1i = Fj_1j;
	ni_1i = Nj_1j + cross(cj_1j, Fj_1j);
	h(j,j)= ni_1i(3,1);
   
   % Elements j-1 to 1. Components H(1:j-1,j)
	i = j - 1;
  	while i >= 1
  		% Vector p.
  		p = [a(i); d(i)*sin(alfa(i)); d(i)*cos(alfa(i))];
          
      % Rotation matrices.
  		ai_1i = dh(teta(i),alfa(i));
		aii_1 = ai_1i';
           
      % Force and torque of the previous element.
  		fiiM1 = fi_1i;
  		niiM1 = ni_1i;
           
      % Strength and torque of this element.
  		fi_1i = ai_1i*fiiM1;
  		ni_1i = ai_1i*(niiM1 + cross(p, fiiM1));
           
      % Component H(i,j).
  		h(i,j)= ni_1i(3,1);
           
      % H is symmetric.
  		h(j,i)= h(i,j);
  		i = i - 1;
  	end
end

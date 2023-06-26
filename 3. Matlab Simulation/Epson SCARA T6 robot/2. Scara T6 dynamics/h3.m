% H3 Moment of inertia matrix. 
% H = H3(Q, MASAEXT, INERCIAEXT) calculates the moment of inertia matrix H 3x3 using the third method of Walker and Orin. 
% H 3x3 using the third method of Walker and Orin. Q is 
% the 3x1 vector of joint variables.  MASAEXT is the mass of the 
% external load. INERCIAEXT is the inertia of the external load.
% 

function  h = h3(q, masaext, inerciaext)

% Denavit-Hartenberg parameters of the robo
teta  = [q(1)    q(2)    0         ];
d     = [0       0      -q(3)-0.083];
a     = [0.325   0.275   0         ];
alfa  = [0       0       0         ];

% Mass of each element (Kg)
m    = [1.6797;  7.5355;  0.08317];

% Matrices of centroidal inertias (Kg-m^2.)
J    = [0.0020 0 -0.0017;0 0.0712 0;-0.0017 0 0.0725;
        0.1351 0 -0.1429;0 0.3854 0;-0.1429 0 0.2658;
        0.0051 0  0     ;0 0.0051 0; 0      0 0     ];

% The fourth inertia is that of the external load.
J(10:12,1:3) = inerciaext;
      
% Vector Z0.
z0 = [0; 0; 1];

% External Load Conditions.
M(4)  = masaext;
cj_1j = zeros(3,1);
Ej_1j = J(10:12,1:3);


for j = 3:-1:1
	%Constant  to extract the Inertia J.
	k = (j-1)*3 + 1;
   %{
   % Vectores p y s.
	p = [a(j); d(j)*sin(alfa(j)); d(j)*cos(alfa(j))];
	s = -0.5*p;
    %}
    %
    if j==3
        p = [0; 0; -0.41];
        s = [0; 0; 0.207];
    end
    
    if j==2
        p = [0.275   ; 0; 0.0025];
        s = [-0.15534; 0; 0.1171];
    end
    
     if j==1
         p = [0.325   ; 0; -0.0029];
         s = [-0.17756; 0; 0.00405];
     end      
    
 %}  
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
   if (j == 1) | (j == 2)	%  Rotational joints
      Fj_1j = cross(z0,M(j)*cj_1j);
      Nj_1j = Ej_1j*z0;
   else	% Prismatic joints
      Fj_1j = M(j)*z0;
      Nj_1j = zeros(3,1);
   end
   
   % Element j. Component H(j,j).
	fi_1i = Fj_1j;
	ni_1i = Nj_1j + cross(cj_1j, Fj_1j);
   if (j == 1) | (j == 2)	% Rotational joints
      h(j,j) = ni_1i(3,1);
   else	% Prismatic joints
      h(j,j) = fi_1i(3,1);
   end
   
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
      if (i == 1) | (i == 2)	% Rotational joints
         h(i,j) = ni_1i(3,1);
      else	% Prismatic joints
         h(i,j) = fi_1i(3,1);
      end
           
      % H is symmetric.
  		h(j,i)= h(i,j);
  		i = i - 1;
  	end
end

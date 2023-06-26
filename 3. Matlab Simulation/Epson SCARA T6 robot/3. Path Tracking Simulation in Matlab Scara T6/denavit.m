function dh = denavit(teta,d,a,alfa)

Tzteta = [cos(teta)  -sin(teta)   0   0;
          sin(teta)   cos(teta)   0   0;
          0           0           1   0;
          0           0           0   1];
     
Tzd = [1  0  0  0;
       0  1  0  0;
       0  0  1  d; 
       0  0  0  1];
 
Txa = [1  0  0  a;
       0  1  0  0;
       0  0  1  0; 
       0  0  0  1];
  
Txalfa = [1  0           0           0;
          0  cos(alfa)  -sin(alfa)   0;
          0  sin(alfa)   cos(alfa)   0;
          0  0            0          1];
      
dh = Tzteta*Tzd*Txa*Txalfa;

end
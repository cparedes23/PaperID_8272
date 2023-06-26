function drawsistema3dh(xi,yi,zi,i)

r = 0.02;  % shaft size
s = 7;     % font size

if i==0
    % Plotting xi, yi, zi
    plot3([xi xi+r], [yi yi  ], [zi zi  ], 'r');      % draw eje xi
    plot3([xi xi  ], [yi yi+r], [zi zi  ], 'g');      % draw eje yi
    plot3([xi xi  ], [yi yi  ], [zi zi+r], 'b');      % draw eje zi

    % Text xi, yi, zi
    text(xi+r, yi  , zi  , 'x'+string(i), 'FontSize', s);   %drax text xi
    text(xi  , yi+r, zi  , 'y'+string(i), 'FontSize', s);   %drax text yi
    text(xi  , yi  , zi+r, 'z'+string(i), 'FontSize', s);   %drax text zi

end

if i==4
    % Plotting xi, yi, zi
    plot3([xi xi+r], [yi yi  ], [zi zi  ], 'r');      % draw eje xi
    plot3([xi xi  ], [yi yi-r], [zi zi  ], 'g');      % draw eje yi
    plot3([xi xi  ], [yi yi  ], [zi zi-r], 'b');      % draw eje zi

    % Text xi, yi, zi
    text(xi+r, yi  , zi  , 'x'+string(i), 'FontSize', s);   %drax text xi
    text(xi  , yi-r, zi  , 'y'+string(i), 'FontSize', s);   %drax text yi
    text(xi  , yi  , zi-r, 'z'+string(i), 'FontSize', s);   %drax text zi

end

if i==1 || i==2 || i==3 || i==5 || i==6
    % Plotting xi, yi, zi
    plot3([xi xi+r], [yi yi  ], [zi zi  ], 'r');      % draw eje xi
    plot3([xi xi  ], [yi yi  ], [zi zi+r], 'g');      % draw eje yi
    plot3([xi xi  ], [yi yi-r], [zi zi  ], 'b');      % draw eje zi

    % Text xi, yi, zi
    text(xi+r, yi  , zi  , 'x'+string(i), 'FontSize', s);   %drax text xi
    text(xi  , yi  , zi+r, 'y'+string(i), 'FontSize', s);   %drax text yi
    text(xi  , yi-r, zi  , 'z'+string(i), 'FontSize', s);   %drax text zi

end


end
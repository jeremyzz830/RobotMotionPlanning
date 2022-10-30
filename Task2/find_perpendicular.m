function [perpendicular_point] = find_perpendicular( point, end_A, end_B)

if end_A(1) == end_B(1) % AB is vertical
    
    perpendicular_point = [ end_A(1); point(2) ];
    
elseif end_A(2) == end_B(2) % AB is horizontal
    
    perpendicular_point = [ point(1); end_A(2) ];
    
else
    
    % set the line between end_A and end_B as y = k(x-x1) + y1
    % the perpendicular line will be y = -1/k ( x - xp) + yp
    k = (end_B(2) - end_A(2)) / (end_B(1) - end_A(1) );
    A = [ -k , 1 ; 1/k, 1];
    b = [ -k*end_A(1) + end_A(2) ; (1/k) * point(1) + point(2) ];
    perpendicular_point = A \ b;
    
    
end

end


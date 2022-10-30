function [ Distance, center] = find_closest_dist( pt, Obs )

Obs = [ Obs , Obs( : , 1) ];

T = [ 1 2 3 ];
p = [cos(pt(3)), -sin(pt(3)) ; sin(pt(3)) cos(pt(3))] * [ -3/2 0; 3/2 0; 0 3*sqrt(3)/2 ];
tr = triangulation(T,p);
center = circumcenter(tr);

[ ~ , Distance ] = ClosestPointOnEdgeToPoint( [ Obs( : , 1 ) , Obs( : , 2 ) ] , center );

for j = 2:1: size(Obs , 2) - 1
    
    [ ~ , distance] = ClosestPointOnEdgeToPoint( [ Obs( : , j ) , Obs( : , j+1 )] , center );
    
    if distance < Distance
        
        Distance = distance;
        
    end
    
end

end

function [closest_pt, distance ] = ClosestPointOnEdgeToPoint( edges, pt)

if  dot( edges(:,1) - pt,  edges(:,2) - edges(:,1) ) > 0
    closest_pt = edges(:,1);
    distance = sqrt( sum((pt - closest_pt) .^ 2));
elseif dot( edges(:,2) - pt, edges(:,1) - edges(:,2) ) > 0
    closest_pt = edges(:,2);
    distance = sqrt( sum((pt - closest_pt) .^ 2));
else
    closest_pt = find_perpendicular(pt, edges(:,1), edges(:,2) );
    distance = sqrt( sum((pt - closest_pt) .^ 2));
    
end

end


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
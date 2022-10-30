function d_u_rep = Repulsive_Gradient( pt , Q , eta , Obs_line)

d_u_rep = [ 0 ; 0 ] ;

for i = 1:1:size(Obs_line, 3)
    
    Obs_i = Obs_line( : , : , i );
    
    e = eta(i);
    
    % find the cloest distance from the control point to ith obstacle
    [ C , D ] = ClosestPointOnEdgeToPoint( [ Obs_i( : , 1 ) , Obs_i( : , 2 ) ] , pt );
    
    for j = 2:1: size(Obs_i , 2) - 1
        
        [ closest_pt , distance] = ClosestPointOnEdgeToPoint( [ Obs_i( : , j ) , Obs_i( : , j+1 )] , pt );
        
        if distance < D
            
            C = closest_pt;
            
            D = distance;
            
        end
        
    end
    
    if D <= Q
        
        d_u_rep = d_u_rep + e * ( 1/Q - 1/D) * ( 1 / D^2 ) * ( pt - C ) / D ;
        
    else
        
        d_u_rep = d_u_rep + [ 0 ; 0 ] ;
        
    end
    
end

end
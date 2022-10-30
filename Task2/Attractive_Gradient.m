function d_u_att = Attractive_Gradient( pt_now , qg, ctrlpts_i , Q , zeta)

% find the coordinates for given control points at goal position
pt_goal = qg( 1 : 2 , : ) +  [ cos( qg(3) ) , -sin( qg(3) ) ; sin( qg(3) ) , cos( qg(3) )] * ctrlpts_i; 

if norm( pt_now - pt_goal ) <= Q
    
    d_u_att = zeta * ( pt_now - pt_goal ) ; 
    
else
    
    d_u_att = Q *  zeta * ( pt_now - pt_goal ) / norm( pt_now - pt_goal ) ;
    
end

end
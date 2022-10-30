function [path] = Artificial_Potential_SE2( qi , qg , X_range , Y_range, O , NumCtrl , StickLength , alpha)

disp("Please wait...");

% set the coefficiences of the potential energy
Q_goal = 10;
Q_obs = 1.5;

for i = 1:1:NumCtrl
    zeta(i) = 0.05;
end

for i = 1:1:size(O,2)
    eta(i) = 0.1;
end
% eta(1) = 0.3;
% eta(3) = 0.3;

delta = [ alpha 0 0 ; 0 alpha 0 ; 0 0 10*alpha ];

% plot the environment and pull out all the line segments in Obs_line
plotstick( qi , StickLength , 'k' , 3 );
axis equal;
plotstick( qg , StickLength , 'r' , 3 );
plot( [ X_range(1), X_range(2), X_range(2), X_range(1), X_range(1)] ,...
    [ Y_range(1), Y_range(1), Y_range(2), Y_range(2), Y_range(1)] , 'LineStyle', '- -');
[ ~, cols] = size(O);
for i = 1:1:cols
    Oi = O{1,i};
    Obs_line( : , : , i ) = [Oi , Oi( : , 1 )];
    Obs = polyshape( [ Oi( 1 , : ) , Oi( 1 , 1 )] , [ Oi( 2 , : ) , Oi( 2 , 1 )] ) ;
    plot(Obs);
end



% compute body coordinates for  every control points
for i = 1:1:NumCtrl
    ctrlpts_in_bodyframe( : , i ) = [ ( -0.5 + ( i - 1 )/ (NumCtrl - 1 ) ) * StickLength ;
        0.0 ];
end

q = qi;
counter = 0;


% Start Gradient Descent
while(true)
    
    u = [ 0 ; 0 ; 0 ];
    
    q_prev = q( : , size(q,2) ); % center point coordinates for previous state
    
    for i = 1:1:NumCtrl
        
        ctrlpts_i = ctrlpts_in_bodyframe( : , i);
        
        pt = q_prev( 1:2 , : ) + [ cos( q_prev(3) ) , -sin( q_prev(3) ) ; sin( q_prev(3) ) , cos( q_prev(3) )] * ctrlpts_i;
        
        J = [ eye(2) , [ -ctrlpts_i(1) * sin(q_prev(3)) ; ctrlpts_i(1) * cos(q_prev(3)) ] ]; % Jacobian for ith control point at q_prev state
        
        d_u_att = Attractive_Gradient( pt , qg , ctrlpts_i , Q_goal , zeta(i) );
        
        d_u_rep = Repulsive_Gradient( pt  , Q_obs, eta, Obs_line);
        
        u = u + J' * ( d_u_att + d_u_rep );
        
    end
    
%     disp(u);
    q_next = q_prev - delta * u;
    
    q = [ q, q_next];
    
    if norm(u) <= 1e-3 && norm( q_next - qg ) < 0.01
        break;
    end
    
    counter = counter + 1;
    if mod( counter , 100 ) == 1
        %         plotstick( q_next, StickLength , 'b' , 1 );

    end
    
    path = q;
    
end

fprintf(' %i steps in total to reach the goal \n', counter);

end

function []=plotstick( q , StickLength , color , Width)

Stick_x = [ q(1) - 0.5 * cos( q(3) ) * StickLength , q(1) + 0.5 * cos( q(3) )* StickLength];

Stick_y = [ q(2) - 0.5 * sin( q(3) )* StickLength , q(2) + 0.5 * sin( q(3) )* StickLength];

plot( Stick_x, Stick_y , color , 'LineWidth' , Width );

hold on;

end
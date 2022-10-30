clear;
clc;
close all;

%% Obstacles
O1 = [ 5 10 10; 3 3 2];
O2 = [ 0 6 0 ; 4 5 6];
O3 = [5 10 10 ; 7 8 5.5];
O4 = [ 0 3 4; 8 8 10 ];

O = cell(1,3);
O{1,1} = O1;
O{1,2} = O2;
O{1,3} = O3;
O{1,4} = O4;

%% initial and goal points in SE2
qi = [ 7 ; 2 ; 0 ];
qg = [ 6 ; 9 ; -0.2 ];

%% Range of the field and step size

X_range = [ 0 , 10 ];
Y_range = [ 0 , 10 ];
step_size = 0.15;

%% Stick Size and Control points
StickLength = 1;
NumCtrl = 5;

%% Artificial Potential Field Method for SE2 scenario

[path] = Artificial_Potential_SE2( qi , qg , X_range , Y_range , O , NumCtrl , StickLength , step_size);

fprintf("\n Path found! \n Drawing results... \n");

%% Plot the result
title("APF for Robot in SE(2)")
Init_txt = '\uparrow Initial State';
text(qi(1),qi(2),Init_txt, 'VerticalAlignment', 'top')
Goal_txt = '\downarrow Goal State';
text(qg(1),qg(2),Goal_txt , 'VerticalAlignment', 'bottom')
pause(1)

StickBoundary = [ - StickLength/2 , - StickLength/2 , StickLength/2 , StickLength/2 , - StickLength/2 ;
    - 0.025 ,                 0.025 ,               0.025 ,            - 0.025 ,               - 0.025 ];
StickBoundary = [ StickBoundary ;  ones(1,5) ] ;

for i = 1:1:size(path,2)
    
    center = path( : , i );
    
    SE2 = [ cos( center(3) ) , -sin( center(3) ) , center(1) ;
        sin( center(3) ) , cos( center(3) ) , center(2) ;
        0 0 1];
    
    Stick = SE2 * StickBoundary;
    
    if (mod( i ,10) == 1) || ( i <= 150 && mod( i , 4) == 1) || i >= size(path,2) - 2
        m = plot( Stick( 1, : ) , Stick( 2, : ) , 'b' );
        pause(0.1)
    end
    
%     if (mod( i ,2) == 1)
%         m = plot( Stick( 1, : ) , Stick( 2, : ) , 'b' );
%         pause(0.02)
%     end
    
    if i ~= size(path,2)
%         set(m, 'Visible','off');
    else
        n = plot( Stick( 1, : ) , Stick( 2, : ) , 'b', 'DisplayName', ' Robot' );
    end
end

legend(n)


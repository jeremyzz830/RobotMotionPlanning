function [path,V,E , restart] = needle_RRT(q_init,q_goal,NumNodes,del_t,Obs,L_world,r,u_w,u_phi)

restart = true;
for i = 1:size(Obs,2)
    temp_Obs = Obs{i};
    temp_Obs(:,size(temp_Obs,2)+1) = temp_Obs(:,1);
    plot(temp_Obs(1,:),temp_Obs(2,:),'LineWidth',2);
    hold on;
end

m = plot(q_init(1),q_init(2),'o');
n = plot(q_goal(1),q_goal(2),'o');
Init_txt = '\uparrow Initial State';
text(q_init(1),q_init(2),Init_txt, 'VerticalAlignment', 'top')
Goal_txt = '\downarrow Goal State';
text(q_goal(1),q_goal(2),Goal_txt , 'VerticalAlignment', 'bottom')
axis equal;

path = [];
V = [];
E = [];

V(:,1) = q_init;
E(:,1) = [1;1];

for i = 1:NumNodes
    % search for (i+1)th node around ith node
    % to set a random nearby point, but only x-y coordinate
    if mod(i,10) ~= 0
        q_rand(1,1) = L_world*rand;       % q_rand 1*2
        q_rand(2,1) = L_world*rand;
    else
        q_rand = q_goal(1:2);
    end
    
    Flag = false;
    %     q_rand(1,1) = L_world*rand;       % q_rand 1*2
    %     q_rand(2,1) = L_world*rand;
    %q_rand = 0.8*q_rand + 0.2*q_goal(1:2);
    while 1
        [q_near,idx] = Nearest_Vertex(q_rand,V);        % q_near 1*3
        
        if Flag == false
             [q_new,u_w_select] = New_Conf(q_near,q_rand,del_t,r,u_phi,u_w , q_near);% q_new 1*3
        else
            [q_parent,parent_idx] = seek_grand(idx , E, V);
            q_new = q_parent + del_t.*[r*cos(q_parent(3)) 0;r*sin(q_parent(3)) 0;0 1]*[u_phi;u_w(3)];
        end
        
        Flag = false;
        
        if ismember( ismember( V', q_new', 'rows'), true )
            q_rand(1,1) = L_world*rand;       % q_rand 1*2
            q_rand(2,1) = L_world*rand;
            continue;
        end
        if q_new(3)<-pi
            q_new(3) = q_new(3) + 2*pi;
        end
        if q_new(3)>pi
            q_new(3) = q_new(3) - 2*pi;
        end
        distance = check_collision(q_near,q_new,Obs);
        if distance > 1.7321    % no collision
            break;
        elseif distance <= 1.7321 && u_w_select >= 0
            Flag = true;
        end
            q_rand(1,1) = L_world*rand;       % q_rand 1*2
            q_rand(2,1) = L_world*rand;

    end
    
    if mod(i,300) == 0
        disp("node index is");
        disp(i);
        disp("distance between q_new & q_goal is");
        l = norm(q_new(1:2)-q_goal(1:2));
        disp(l);
        disp("please wait...");
        disp("############################");
    end
    
    V(:,i+1) = q_new; % 1*3
    E(2,i+1) = i+1; % new/son point's index
    E(1,i+1) = idx; % father point's index
    
    if norm(q_new(1:2)-q_goal(1:2))<0.8
        path(:,1) = q_new;
        idx_son = i+1;
%         disp("SUCCESS !!!");
        disp("distance is");
        l = norm(q_new(1:2)-q_goal(1:2));
        disp(l);
        restart = false;
        break;
    end
    
    if i == NumNodes-1
        return;
    end
end

% path
while path(1,size(path,2)) ~= q_init(1,1)
    x = find(E(2,:) == idx_son);
    idx_dad = E(1,x);
    node_dad = V(:,idx_dad);
    path(:,size(path,2)+1) = node_dad;
    idx_son = idx_dad;
end
path = fliplr(path);

plot(path(1,:),path(2,:),'--r','LineWidth',1);
m = plot(q_init(1),q_init(2),'o');
n = plot(q_goal(1),q_goal(2),'o');
axis equal;
% plot trajectory
for i = 1:size(path,2)
    temp_x = path(1,i);
    temp_y = path(2,i);
    temp_theta = path(3,i);
    robot = [temp_x+1.5*sin(temp_theta) temp_x+1.5*3^0.5*cos(temp_theta) temp_x-1.5*sin(temp_theta);temp_y-1.5*cos(temp_theta) temp_y+1.5*3^0.5*sin(temp_theta) temp_y+1.5*cos(temp_theta)];
    l = plot([robot(1,:) robot(1,1)],[robot(2,:) robot(2,1)],'-b');
    gca = legend([m n],{'start point','goal point'});
    set(gca,'Position',[0 40 10 10]);
    pause(0.05);
    if i ~=  size(path,2)
        set(l,'Visible','off');
    end
end
end

function [q_near,idx] = Nearest_Vertex(q,V)
d = Inf;
for i = 1:size(V,2)
    if norm(q-V(1:2,i))<d
        q_near = V(:,i);      % 1*3
        d = norm(q-V(1:2,i));
        idx = i;
    end
end
end

function [q_new,u_w_select] = New_Conf(q_near,q_rand,del_t,r,u_phi,u_w , q_parent)
alpha = atan2(q_rand(2)-q_near(2),q_rand(1)-q_near(1));
theta = q_near(3);
if abs(theta - alpha)<=0.3
    u_w_select = u_w(2);
    q_new = q_near + del_t.*[r*cos(theta) 0;r*sin(theta) 0;0 1]*[u_phi;u_w_select];
elseif theta > alpha
    u_w_select = u_w(3);
    q_new = q_near + del_t.*[r*cos(theta) 0;r*sin(theta) 0;0 1]*[u_phi;u_w_select];
else
    u_w_select = u_w(1);
    q_new = q_near + del_t.*[r*cos(theta) 0;r*sin(theta) 0;0 1]*[u_phi;u_w_select];
end
end

function dist = check_collision(q_near,q_new,Obs)
num_Obs = size(Obs,2);
num = 10;
dis = [];
for j = 1:num
    for i = 1:num_Obs
        temp = q_near + j/num*(q_new-q_near);
        temp_x = temp(1);
        temp_y = temp(2);
        temp_theta = temp(3);
        distance = find_closest_dist([temp_x;temp_y;temp_theta],Obs{i});
        dis = [dis distance];
    end
end
dist = min(dis);
end

function [ Distance, center] = find_closest_dist( pt, Obs )

Obs = [ Obs , Obs( : , 1) ];

T = [ 1 2 3 ];
p = [cos(pt(3)), -sin(pt(3)) ; sin(pt(3)) cos(pt(3))] * [ 0 -3/2; 0 3/2; 3*sqrt(3)/2 0]';
tr = triangulation(T,p');
center = circumcenter(tr) + [pt(1) , pt(2)];

[ ~ , Distance ] = ClosestPointOnEdgeToPoint( [ Obs( : , 1 ) , Obs( : , 2 ) ] , center' );

for j = 2:1: size(Obs , 2) - 1
    
    [ ~ , distance] = ClosestPointOnEdgeToPoint( [ Obs( : , j ) , Obs( : , j+1 )] , center' );
    
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

function [grand,idx_grand] = seek_grand(idx_son,E,V)
    temp = find(E(2,:) == idx_son);
    idx_dad = E(1,temp);
    temp = find(E(2,:) == idx_dad);
    idx_grand = E(1,temp);
    grand = V(:,idx_grand);     %3*1
end
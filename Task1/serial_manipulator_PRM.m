% clc ;
% clear;
% close all;
% q_init = [0;0;0;0];              % configuration init
% q_goal = [2;2.1;1.9;1.7];
% % q_goal = [q_goal(1);q_goal(1)+q_goal(2);q_goal(1)+q_goal(2)+q_goal(3);q_goal(1)+q_goal(2)+q_goal(3)+q_goal(4)];
% NumNodes = 1000;
% K = 4;
% L = 8;      % length of one section of manipulator
% 
% %%%%%% set obs in worksapce
% Obs = {[5 25 25 5;20 20 35 35], [-30 -10 -10 -30;-25 -25 10 10], [-8 30 30 -8; -25 -25 -5 -5]};
% 
% [path,V,G] = serial_manipulator_PRM(q_init,q_goal,NumNodes,K,Obs,L);

function [path,V,G] = serial_manipulator_PRM(q_I,q_G,n,K,Obs,L)
% â€? qI and qG: initial and goal position of the robot.
% â€? n: the limit number of nodes.
% â€? K: number of nearest neighbors.
% â€? Obs is the cell array of obstacles.
% â€? (xmax, ymax) the coordinates of the upper right corner of the workspace box (the origin is
% the lower left corner as in P1).

V = [];     % 2*~
E = [];     % 2*~
G = [];
for i = 1:n
    for j = 1:n
        if i==j
           G(i,j) = 0; 
        end
        G(i,j) = Inf;
    end
end
count = 0;

while size(V,2)<n
    % to produce #count point
    count = count + 1;
    q_rand = 2*pi*rand(4,1);
    while 1
        res_logi = check_collision(q_rand,Obs,L);
        if  res_logi == 0
            break;
        end
        q_rand = 2*pi*rand(4,1);
    end
    V(:,count) = q_rand;
end

for i = 1:n
    temp_V = V;     % to generate a temporary V for deleting nodes conveniently
    N_q = [];
    % to generate k num neighbor points around ith node
    for j = 1:K
        [q_near,idx] = Nearest_Vertex(V(:,i),temp_V);
        temp_V(:,idx) = [];     % to delete the selected node from temp_V
        N_q(:,j) = q_near;      % to add the neighbor node into N_q
        res_logi = check_collision_dual(V(:,i),q_near,Obs,L);
        
        if res_logi == 0       % no collision
            idx_V = find(V(1,:) == q_near(1,1));
            E = [E [i;idx_V]];
            G(i,idx_V) = 1;
            G(idx_V,i) = 1;
        end
    end    
end

for i = 1:n
    G(i,i) = 0;
end

% to connect the initial point to roadmap
temp_V_1 = V;
while 1
    [q_near_init,idx_init] = Nearest_Vertex(q_I,temp_V_1);
    res_logi = check_collision_dual(V(:,i),q_near,Obs,L);
    if res_logi == 0       % no collision
        path(:,1:2) = [q_I q_near_init];
        break;
    end 
    temp_V_1(:,idx_init) = [];
end

% to connect the goal point to roadmap
temp_V_2 = V;
while 1
    [q_near_goal,idx_goal] = Nearest_Vertex(q_G,temp_V_2);
    res_logi = check_collision_dual(V(:,i),q_near_goal,Obs,L);
    if res_logi == 0       % no collision
        path(:,3:4) = [q_near_goal q_G];
        break;
    end 
    temp_V_2(:,idx_goal) = [];
end

% to get the mid path
mid_path = [];
[path_idx,~] = DijkstraAlgorithm (G,idx_init,idx_goal);
for i = 1:size(path_idx,1)
    mid_path = [mid_path V(:,path_idx)];
end

path = [path(:,1:2) mid_path path(:,3:4)];

% plot
for i = 1:size(Obs,2)
    temp_Obs = Obs{i};
    temp_Obs(:,size(temp_Obs,2)+1) = temp_Obs(:,1);
    plot(temp_Obs(1,:),temp_Obs(2,:),'LineWidth',0.5);
    hold on;
end

End = [];
for i = 1:size(path,2)
     End_new = trans_CW(path(:,i),L);
     End = [End End_new];
     plot(End_new(1),End_new(2),'-o','MarkerSize',4);
     hold on;
     h = plot(End_new(3),End_new(4),'-o','MarkerSize',4);
     j = plot(End_new(5),End_new(6),'-o','MarkerSize',4);
     k = plot(End_new(7),End_new(8),'-o','MarkerSize',4);
     l = plot(End_new(9),End_new(10),'-o','MarkerSize',4);
     W_goal = trans_CW(q_G,L);
     u =  plot([W_goal(1) W_goal(3) W_goal(5) W_goal(7) W_goal(9)],[W_goal(2) W_goal(4) W_goal(6) W_goal(8) W_goal(10)],'r');
     m = plot([End_new(1) End_new(3) End_new(5) End_new(7) End_new(9)],[End_new(2) End_new(4) End_new(6) End_new(8) End_new(10)],'b');
     legend([u m],{'goal position','current position'});
     pause(0.2)
     if i ~=  size(path,2)
        set(h,'Visible','off');
        set(j,'Visible','off');
        set(k,'Visible','off');
        set(l,'Visible','off');
        set(m,'Visible','off');
     end
end

end

function [q_near,idx] = Nearest_Vertex(q,V)
    d = Inf;
    for i = 1:size(V,2)
        if norm(q-V(:,i))<d
            q_near = V(:,i);
            d = norm(q-V(:,i));
            idx = i;
        end
    end
end

function [workspace] = trans_CW(C_space,L)
    % return 10*1 array
    j1 = [0;0]; % origin of the serial manipulator
    j2 = [j1(1)+L*cos(C_space(1));j1(2)+L*sin(C_space(1))];
    j3 = [j2(1)+L*cos(C_space(2));j2(2)+L*sin(C_space(2))];
    j4 = [j3(1)+L*cos(C_space(3));j3(2)+L*sin(C_space(3))];
    j5 = [j4(1)+L*cos(C_space(4));j4(2)+L*sin(C_space(4))];
    workspace = [j1;j2;j3;j4;j5];
end

function [res] = check_collision_dual(C_start,C_final,Obs,L)
    % C_start C_final are 1*4 array
    num = 3;           % number of middle points
    res = 0;
    num_obs = size(Obs,2);
    
    for k = 1:num_obs
        temp_obs = Obs{k};
        for j = 1:num
            C_temp = C_start+ 1/num*j*(C_final - C_start);        
            temp = trans_CW(C_temp,L);            % 10*1 array
            temp_1 = [temp(1:2)+0.2*(temp(3:4)-temp(1:2));temp(3:4)+0.2*(temp(5:6)-temp(3:4));temp(5:6)+0.2*(temp(7:8)-temp(5:6));temp(7:8)+0.2*(temp(9:10)-temp(7:8))]; 
            temp_2 = [temp(1:2)+0.4*(temp(3:4)-temp(1:2));temp(3:4)+0.4*(temp(5:6)-temp(3:4));temp(5:6)+0.4*(temp(7:8)-temp(5:6));temp(7:8)+0.4*(temp(9:10)-temp(7:8))]; 
            temp_3 = [temp(1:2)+0.6*(temp(3:4)-temp(1:2));temp(3:4)+0.6*(temp(5:6)-temp(3:4));temp(5:6)+0.6*(temp(7:8)-temp(5:6));temp(7:8)+0.6*(temp(9:10)-temp(7:8))]; 
            temp_4 = [temp(1:2)+0.8*(temp(3:4)-temp(1:2));temp(3:4)+0.8*(temp(5:6)-temp(3:4));temp(5:6)+0.8*(temp(7:8)-temp(5:6));temp(7:8)+0.8*(temp(9:10)-temp(7:8))]; 
            for i = 1:5
                if inpolygon(temp(2*i-1,1),temp(2*i,1),temp_obs(1,:),temp_obs(2,:))
                    res = res + 1;
                    break;
                end
            end
            for i = 1:4
                if inpolygon(temp_1(2*i-1,1),temp_1(2*i,1),temp_obs(1,:),temp_obs(2,:)) || inpolygon(temp_2(2*i-1,1),temp_2(2*i,1),temp_obs(1,:),temp_obs(2,:)) || inpolygon(temp_3(2*i-1,1),temp_3(2*i,1),temp_obs(1,:),temp_obs(2,:)) || inpolygon(temp_4(2*i-1,1),temp_4(2*i,1),temp_obs(1,:),temp_obs(2,:))
                    res = res + 1;
                    break;
                end
            end
        end
        if res ~= 0
            break;
        end
    end     
end

function [res] = check_collision(C_space,Obs,L)
    res = 0;
    num_obs = size(Obs,2);
    for k = 1:num_obs
        temp_obs = Obs{k};    
        temp = trans_CW(C_space,L);            % 10*1 array
        temp_1 = [temp(1:2)+0.2*(temp(3:4)-temp(1:2));temp(3:4)+0.2*(temp(5:6)-temp(3:4));temp(5:6)+0.2*(temp(7:8)-temp(5:6));temp(7:8)+0.2*(temp(9:10)-temp(7:8))]; 
        temp_2 = [temp(1:2)+0.4*(temp(3:4)-temp(1:2));temp(3:4)+0.4*(temp(5:6)-temp(3:4));temp(5:6)+0.4*(temp(7:8)-temp(5:6));temp(7:8)+0.4*(temp(9:10)-temp(7:8))]; 
        temp_3 = [temp(1:2)+0.6*(temp(3:4)-temp(1:2));temp(3:4)+0.6*(temp(5:6)-temp(3:4));temp(5:6)+0.6*(temp(7:8)-temp(5:6));temp(7:8)+0.6*(temp(9:10)-temp(7:8))]; 
        temp_4 = [temp(1:2)+0.8*(temp(3:4)-temp(1:2));temp(3:4)+0.8*(temp(5:6)-temp(3:4));temp(5:6)+0.8*(temp(7:8)-temp(5:6));temp(7:8)+0.8*(temp(9:10)-temp(7:8))]; 
        for i = 1:5
            if inpolygon(temp(2*i-1,1),temp(2*i,1),temp_obs(1,:),temp_obs(2,:))
                res = res + 1;
                break;
            end
        end
        for i = 1:4
            if inpolygon(temp_1(2*i-1,1),temp_1(2*i,1),temp_obs(1,:),temp_obs(2,:)) || inpolygon(temp_2(2*i-1,1),temp_2(2*i,1),temp_obs(1,:),temp_obs(2,:)) || inpolygon(temp_3(2*i-1,1),temp_3(2*i,1),temp_obs(1,:),temp_obs(2,:)) || inpolygon(temp_4(2*i-1,1),temp_4(2*i,1),temp_obs(1,:),temp_obs(2,:))
                res = res + 1;
                break;
            end
        end
    end
end


function [path,final_distance] = DijkstraAlgorithm (graph,n_init,n_goal)

m = size(graph,1); % to record the number of all nodes
dist(1:m) = Inf;
prev = zeros(1,m);
U = zeros(1,m);  % unvisited nodes set
dist(n_init) = 0;  %initialize the n_init node with distance 0

while U(n_goal) ~= 1
    candidate_nodes (1:m) = Inf;
    for i = 1:m
        if U(i) ==0  % to seek unvisited nodes
            candidate_nodes(i) = dist(i);
        end
    end
    
    [C_dist,C_node] = min(candidate_nodes);
    U(C_node) = 1; % to notate the visited node
    for i = 1:m
        if graph(C_node,i) ~= Inf && graph(C_node,i) ~= 0  % find the neighbor node
        alt = C_dist+graph(C_node,i);
        if alt<dist(i)
            dist(i) = alt;
            prev(i) = C_node;
        end
        end
    end
end

i = n_goal;
j = 1;
path = (n_goal);
while i ~= n_init
     i = prev (i);
     path (j+1) = i;
     j = j+1;
end
path = fliplr (path);
final_distance = dist(n_goal);
end
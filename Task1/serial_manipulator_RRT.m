% clc ;
% clear;
% close all;
% q_init = [0;0;0;0];              % configuration init
% q_goal = [2;2.1;1.9;1.7];
% % q_goal = [q_goal(1);q_goal(1)+q_goal(2);q_goal(1)+q_goal(2)+q_goal(3);q_goal(1)+q_goal(2)+q_goal(3)+q_goal(4)];
% NumNodes = 5e5;
% del_q = 0.05;                       % i.e 1/20 
% L = 8;      % length of one section of manipulator
% 
% %%%%%% set obs in worksapce
% Obs = {[5 25 25 5;20 20 35 35], [-30 -10 -10 -30;-25 -25 10 10], [-8 30 30 -8; -25 -25 -5 -5]};
% 
% [path,V,E] = serial_manipulator_RRT(q_init,q_goal,NumNodes,del_q,Obs,L);

function [path,V,E] = serial_manipulator_RRT(q_init,q_goal,NumNodes,del_q,Obs,L)
% workspace is 2-D model; c-space is 4-D model, i.e 4 joints manipulator
% we need transformation between C-space & workspace
% del_q: step size(const)
% Obs:cell array of obstacles (2,~)
% environment limit: x_max*y_max
% path: R2*N, including q_init and q_goal
% V E:set of vertices and edges

% initialize path 2*N, V: set of all vertices 2*m, E: set of all edges 2*(m-1), 
% 1st row for dad nodes, 2nd for son nodes, only record end node, i.e father node is
% the former node of son node in tree
path = [];
V = [];     % in C-space
E = [];

V(:,1) = q_init;
E(:,1) = [0;1];

for i = 1:NumNodes
    % search for (i+1)th node around ith node
    % to set a random nearby point
    if mod(i,10) ~= 0
        C_q_rand = 2*pi*rand(4,1);
    else
        C_q_rand = q_goal;
    end
    while 1
        temp_end = C_q_rand;
        [temp_start,idx] = Nearest_Vertex(temp_end,V);
        temp_end_new = New_Conf(temp_start,temp_end,del_q);
        res_logi = check_collision(temp_start,temp_end_new,Obs,L);
%         res_logi = 0;
        if res_logi == 0       % no collision
            q_new = temp_end_new;
            break;
        end
        C_q_rand = 2*pi*rand(4,1);
    end

    
    % to search the nearby point in tree around q_rand
    % [q_near,idx] = Nearest_Vertex(C_q_rand,V);
    
    % to get the q_new from q_near, direction from q_near to q_rand, and del_q(stepsize) 
    % q_new = New_Conf(q_near,C_q_rand,del_q);
    
    V(:,i+1) = q_new;
    E(2,i+1) = i+1; % new/son point's index
    E(1,i+1) = idx; % father point's index
    
    if mod(i,100) == 0
        disp("node index is");
        disp(i);
        disp("end joint worksapce distance is");
        d = trans_CW(q_new,L)-trans_CW(q_goal,L);
        d = d(9:10);
        d = norm(d);
        disp(d);
        disp("total configuration distance is");
        l = norm(q_new-q_goal);
        disp(l);
        disp("end joint position is");
        pos = trans_CW(q_new,L);
        pos = pos(9:10);
        disp(pos);
        disp("please wait...");
        disp("############################");
    end
    
    if norm(q_new-q_goal)<0.01
        path(:,1) = q_new;
        disp("SUCCESS !!!");
        disp("end joint worksapce distance is");
        d = trans_CW(q_new,L)-trans_CW(q_goal,L);
        d = d(9:10);
        d = norm(d);
        disp(d);
        disp("total configuration distance is");
        l = norm(q_new-q_goal);
        disp(l);
        break;
    end
    
end

% path
while path(1,size(path,2)) ~= q_init(1,1)
    idx_son = find(V(2,:) == path(2,size(path,2)));
    idx_dad = E(1,find(E(2,:) == idx_son));
    node_dad = V(:,idx_dad);
    path(:,size(path,2)+1) = node_dad;
end
path = fliplr(path);

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
     W_goal = trans_CW(q_goal,L);
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

function [workspace] = trans_CW(C_space,L)
    % return 10*1 array
    j1 = [0;0]; % origin of the serial manipulator
    j2 = [j1(1)+L*cos(C_space(1));j1(2)+L*sin(C_space(1))];
    j3 = [j2(1)+L*cos(C_space(2));j2(2)+L*sin(C_space(2))];
    j4 = [j3(1)+L*cos(C_space(3));j3(2)+L*sin(C_space(3))];
    j5 = [j4(1)+L*cos(C_space(4));j4(2)+L*sin(C_space(4))];
    workspace = [j1;j2;j3;j4;j5];
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

function [q_new] = New_Conf(q_near,q_rand,del_q)
    q_new = q_near + del_q*(q_rand-q_near);
end

function [res] = check_collision(C_start,C_final,Obs,L)
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
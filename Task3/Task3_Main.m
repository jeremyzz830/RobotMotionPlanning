clc ;
clear;
close all;
q_init = [0;0;0];              % configuration init, 1*3
q_goal = [35;45;0];
NumNodes = 1500;
del_t = 0.5;
L_world = 50;
r = 2/pi;
u_phi = 1;
u_w = [pi/7 0 -pi/7];

Obs = {[7 15 15 7;10 10 30 30], [30 40 40 30;20 20 40 40]};

restart = true;


while true
    [path,V,E, restart] = needle_RRT(q_init,q_goal,NumNodes,del_t,Obs,L_world,r,u_w,u_phi);
    if restart == true
        disp("############################");
        fprintf("\n Planning Failed . Restarting the whole process .\n.\n.\n.")
    else
        fprintf("\n Planning Success ! \n")
        disp("############################");
        break
    end
end
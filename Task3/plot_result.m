% plot obs
for i = 1:size(Obs,2)
    temp_Obs = Obs{i};
    temp_Obs(:,size(temp_Obs,2)+1) = temp_Obs(:,1);
    Obs_poly = polyshape( temp_Obs(1,:) , temp_Obs(2,:));
    plot(Obs_poly);
    hold on;
end
plot(path(1,:),path(2,:),'--b','LineWidth',1);
axis equal;
% plot trajectory
for i = 1:size(path,2)
    temp_x = path(1,i);
    temp_y = path(2,i);
    temp_theta = path(3,i);
    robot = [temp_x+1.5*sin(temp_theta) temp_x+1.5*3^0.5*cos(temp_theta) temp_x-1.5*sin(temp_theta);temp_y-1.5*cos(temp_theta) temp_y+1.5*3^0.5*sin(temp_theta) temp_y+1.5*cos(temp_theta)];
    l = plot([robot(1,:) robot(1,1)],[robot(2,:) robot(2,1)],'-r');
    pause(0.05);
     if i ~=  size(path,2)
        set(l,'Visible','off');
     end
end
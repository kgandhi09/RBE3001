% RBE3001 Team 7 Lab 2 MATLAB Code
% FK of the RBE3001 Arm:

% Takes in a matrices of joint encoder pos and velocities, and an index,
% and plots a stick figure version of the robot
function arm_plot(index,pos,vel)
    % convert encoder values to radians
    joint0 = -1*enc2rad(pos(index,2));
    joint1 = -1*enc2rad(pos(index,3));
    joint2 = -1*enc2rad(pos(index,4));

    % XYZ coordinates of all j0, j1, j2, and the end effector
    point0 = [30,0,0];
    point1 = [30,0,135];
    [point2,point3] = fwkin3001(joint0,joint1,joint2);

    % creates a matrix for each coordinate direction for the 4 points
    % matrices will be used in 3d plotting the robot 
    matrixX = [point0(1),point1(1),point2(1),point3(1)];
    matrixY = [point0(2),point1(2),point2(2),point3(2)];
    matrixZ = [point0(3),point1(3),point2(3),point3(3)];
    
    % Plot the Robot
    figure(1)
    plot3(matrixX,matrixY,matrixZ,'o-','LineWidth',2);
    hold on
    grid on
    % limits of this plot
    xlim([0 550]); 
    ylim([-300 300]); 
    zlim([-100 550]);
    title('Live Plot of Robot')
    xlabel('X Position (mm)')
    ylabel('Y Position (mm)')
    zlabel('Z Position (mm)')
    view([45 45])
    hold off
    
    % log joint angle (rad) data and tip position (mm) into a CSV
    output = [pos(index,1), joint0, joint1, joint2, point3(1), point3(2), point3(3)];
    dlmwrite('logJATP.csv',output,'-append');
end

% RBE3001 Team 7 Lab 2 MATLAB Code

% FK of the RBE3001 Arm:
function arm_plot(index,pos)
    % test input
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
    % Plot the Vectors (Links) btwn Points
    figure(1)
    plot3(matrixX,matrixY,matrixZ,'o-','LineWidth',2);
    hold on
    plot3(250,-90,200,'d','Color','r','MarkerSize',10)
    plot3(232.34,99.2,-14.154,'d','Color','b','MarkerSize',10)
%     plot3(170,0,39,'d','Color','g','MarkerSize',10)
%     plot3(148,83,-39,'d','Color','k','MarkerSize',10)
%     plot3(148,-83,-39,'d','Color','m','MarkerSize',10)
    legend('Robot','1st Setpoint','2nd Setpoint')
    xlim([-100 300]); % change back to 500
    ylim([-300 300]); % change back to 400
    zlim([-100 400]); % change back to 550
    grid on
    title('Live Plot of Robot')
    xlabel('X Position (mm)')
    ylabel('Y Position (mm)')
    zlabel('Z Position (mm)')
    hold off
    
    % log data into a CSV
    output = [pos(index,1), joint0, joint1, joint2, point3(1), point3(2), point3(3)];
    disp(point3(2))
    dlmwrite('logJATP.csv',output,'-append');
end

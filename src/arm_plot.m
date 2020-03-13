% RBE3001 Team 7 Lab 2 MATLAB Code

% FK of the RBE3001 Arm:
function arm_plot(index,pos,vel)
    % convert encoder values to radians
    joint0 = -1*enc2rad(pos(index,2));
    joint1 = -1*enc2rad(pos(index,3));
    joint2 = -1*enc2rad(pos(index,4));
    vel = -enc2rad(vel);
    
    curr_jacob = jacob0([joint0,joint1,joint2])
    curr_jacob = curr_jacob(1:3,1:3);
    
    p_vel = fwvelkin([joint0,joint1,joint2],...
                     [vel(index,2), vel(index,3),vel(index,4)]);

    % XYZ coordinates of all j0, j1, j2, and the end effector
    point0 = [30,0,0];
    point1 = [30,0,135];
    [point2,point3] = fwkin3001(joint0,joint1,joint2);

    % creates a matrix for each coordinate direction for the 4 points
    % matrices will be used in 3d plotting the robot 
    matrixX = [point0(1),point1(1),point2(1),point3(1)];
    matrixY = [point0(2),point1(2),point2(2),point3(2)];
    matrixZ = [point0(3),point1(3),point2(3),point3(3)];
    
    % manipulability ellipsoid
    A = curr_jacob * curr_jacob.';
    eigenA = eig(A);
    vol = (4/3)*pi*sqrt(eigenA(1))*sqrt(eigenA(2))*sqrt(eigenA(3));

    % Plot the Robot
    figure(1)
    plot3(matrixX,matrixY,matrixZ,'o-','LineWidth',2);
    hold on
%     plot_ellipse(A, [point3(1),point3(2),point3(3)]);
    grid on
%     if vol < 7.5e09
%         text(0, 0, 100, 'ERROR: Approaching Singularity!', ...
%              'FontSize',14,'Color','red');
%         error('Approaching Singularity! Program Stopped');
%     end
    quiver3(point3(1),point3(2),point3(3), p_vel(1), p_vel(2),p_vel(3));
    legend('Robot', 'Velocity Vector');
    % limits of this plot
    xlim([0 550]); % change back to 500
    ylim([-300 300]); % change back to 400
    zlim([-100 550]); % change back to 550
    title('Live Plot of Robot')
    xlabel('X Position (mm)')
    ylabel('Y Position (mm)')
    zlabel('Z Position (mm)')
    view([45 45])
    hold off
    
    % log data into a CSV
    output = [pos(index,1), joint0, joint1, joint2, point3(1), point3(2), point3(3)];
    dlmwrite('logJATP.csv',output,'-append');
end

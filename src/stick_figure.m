% edits to arm_plot so that it takes in an array of joint angles (radians)
% to plot

% FK of the RBE3001 Arm:
% RBE3001 Team 7 Lab 2 MATLAB Code

% FK of the RBE3001 Arm:
function stick_figure(joints)
    % convert encoder values to radians
    joint0 = -joints(1);
    joint1 = -joints(2);
    joint2 = -joints(3)
    
    curr_jacob = jacob0([joint0,joint1,joint2])
    curr_jacob = curr_jacob(1:3,1:3);

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
    vol = (4/3)*pi*det(A);

    % Plot the Robot
    figure(1)
    plot(matrixX,matrixZ,'o-','LineWidth',2);
    hold on
%     plot_ellipse(A, [point3(1),point3(2),point3(3)]);
    grid on
%     if vol < 7.5e09
%         text(0, 0, 100, 'ERROR: Approaching Singularity!', ...
%              'FontSize',14,'Color','red');
%         error('Approaching Singularity! Program Stopped');
%     end
    legend('Robot');
    % limits of this plot
    xlim([0 550]); % change back to 500
%     ylim([-300 300]); % change back to 400
    zlim([-100 550]); % change back to 550
    title('Live Plot of Robot')
    xlabel('X Position (mm)')
%     ylabel('Y Position (mm)')
    zlabel('Z Position (mm)')
%     view([45 45])
    hold off
end

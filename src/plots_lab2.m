% Plots Everything in Step 7 of Lab 2, Plots
% joint angles(degrees) vs. time(seconds)
% x and z tip position(mm) vs. time(seconds)
% velocity vs time(seconds)

function plots_lab2(table)
% omit the 1st row of data (joint angles are all 0s)
timestamps = table(2:end,1);

% joint angles(degrees) vs. time(seconds)
%     figure(3)
%     % convert to degrees
%     j0_log = table(2:end,2)*(180/pi);
%     j1_log = table(2:end,3)*(180/pi);
%     j2_log = table(2:end,4)*(180/pi);
%     plot(timestamps,j0_log,'r',timestamps,j1_log,'b',timestamps,j2_log,'g');
%     title('Joint Angles vs. Time');
%     xlabel('Time (s)');
%     ylabel('Joint Angles (degrees)');
%     legend('Joint 0','Joint 1', 'Joint 2');

% x, y, and z tip position(mm) vs. time(seconds)
figure(3)
x_log = table(2:end,5);
y_log = table(2:end,6);
z_log = table(2:end,7);
plot(timestamps,x_log,'r', timestamps, y_log, 'k',timestamps,z_log,'m');
title('X, Y, and Z Tip Positions vs. Time');
xlabel('Time (s)');
ylabel('X, Y, and Z Tip Positions (mm)');
legend('X Position','Y Position','Z Position');

% velocity vs time(seconds)
figure(4)
%     j0_vel_log = diff(j0_log);
%     j1_vel_log = diff(j1_log);
%     j2_vel_log = diff(j2_log);
%     % jN_vel_log has 1 less element in it than than jN_log
%     plot(timestamps(1:end-1),j0_vel_log,'r',timestamps(1:end-1),j1_vel_log,'b',...
%          timestamps(1:end-1),j2_vel_log,'g');
%     title('Joint Velocities vs. Time');
%     xlabel('Time (s)');
%     ylabel('Joint Velocities (deg/s)');
%     legend('Joint 0','Joint 1', 'Joint 2');

% Triangle Drawing Path - LAB 2
figure(5)
% plot the 3 setpoints
%     hold on
%     plot(237,260,'ro','MarkerSize',10);
%     plot(186,135,'bo','MarkerSize',10);
%     plot(170,-38,'go','MarkerSize',10);
%     % plot the path
%     plot(x_log, z_log,'k:','LineWidth',2);
%     title('Drawing the Triangle: Path of Tip Positions in the X-Z Plane');
%     xlabel('X Position (mm)');
%     ylabel('Z Position (mm)');
%     legend('1st Setpoint','2nd Setpoint','3rd Setpoint','Tip Position')
%     hold off

% Triangle Drawing Path - LAB 3
%     plot3(175,0,-35,'ro','MarkerSize',10);
%     hold on
%     plot3(232.34,99.2,-14.154,'bo','MarkerSize',10);
%     plot3(150,25,80,'go','MarkerSize',10);
plot3(x_log,y_log,z_log,'k:','LineWidth',2)
title('Drawing the Triangle: Path of Tip Positions in the X-Z Plane');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z Position (mm)');
legend('Tip Position');
grid on
title('Draw Triangle in 3D');

% XYZ Tip Velocities
figure(6)
x_vel_log = diff(x_log);
y_vel_log = diff(y_log);
z_vel_log = diff(z_log);
plot(timestamps(1:end-1),x_vel_log,'r',timestamps(1:end-1),y_vel_log,'k',...
     timestamps(1:end-1),z_vel_log,'m');
title('X, Y, and Z Tip Velocities vs. Time');
xlabel('Time (s)');
ylabel('Tip Velocity (mm/s)');
legend('X Velocity','Y Velocity', 'Z Velocity');

% XYZ Tip Acceleration
figure(7)
x_acc_log = diff(x_vel_log);
y_acc_log = diff(y_vel_log);
z_acc_log = diff(z_vel_log);
plot(timestamps(2:end-1),x_acc_log,'r',timestamps(2:end-1),y_acc_log,'k',...
     timestamps(2:end-1),z_acc_log,'m');
title('X, Y, and Z Tip Velocities vs. Time');
xlabel('Time (s)');
ylabel('Tip Acceleration (mm/s^2)');
legend('X Acceleration','Y Acceleration','Z Acceleration');

end

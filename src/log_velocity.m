function x = log_velocity (index,pos,time_diff)
    vel_j0 = diff(pos(:,2));
      vel_j1 = diff(pos(:,3));
      vel_j2 = diff(pos(:,4));
      
      % create arrays of velocities in deg/sec
      enc2degPerSec = 360/(4095 * time_diff);
      disp(index);
      disp(vel_j1(index,1));
      vel_log_j0(index,1) = elapsed;
      vel_log_j0(index,2) = vel_j0(index)*enc2degPerSec;
      vel_log_j1(index,1) = elapsed;
      vel_log_j1(index,2) = vel_j1(index)*enc2degPerSec;
      vel_log_j2(index,1) = elapsed;
      vel_log_j2(index,2) = vel_j2(index)*enc2degPerSec;
      
      % plot the velocity of each joint
      figure(3)
      plot(vel_log_j0(:,1),vel_log_j0(:,2), 'c', ...
           vel_log_j1(:,1),vel_log_j1(:,2), 'b', ...
           vel_log_j2(:,1),vel_log_j2(:,2)), 'g';

      if abs(vel_log_j0(index,2)) < 1 && abs(vel_log_j1(index,2)) < 1 ... 
         && abs(vel_log_j2(index,2)) < 1
         stopped = 1;
         disp('Stopped!')
      else
         stopped = 0;
      end
end
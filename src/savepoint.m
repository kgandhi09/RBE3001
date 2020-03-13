%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 
try
  SERV_ID = 01;
  STATUS_ID = 03;            % we will be talking to server ID 01 on
                           % the Nucleo

  DEBUG   = true;          % enable output(index,4) = point3;s/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
 
  %initialize matrices
  m = zeros(25,15);
  n = zeros(25,15);
  l = zeros(25,15);
  
  pos = zeros(300,4);
  index = 1;
  prev_time = 0;
  
  timer = clock;

  delete logJATP.csv;
  delete trajectoryj0.csv;
  delete trajectoryj1.csv;
  delete trajectoryj2.csv;
  trajectoryj0 = zeros(20000,5);
  trajectoryj1 = zeros(20000,5);
  trajectoryj2 = zeros(20000,5);
  
    % initial joint values
    
%     q0 = [0,0,0,0];
%     q1 = [0,67.2,56,0];
%     q2 = [0,12.92,-24,0];
    
    q00 = 0;
    q01 = 0;
    q02 = 0;
    q03 = 0;
    
    q10 = 0;
    q11 = 67.2;
    q12 = 56;
    q13 = 0;
    
    q20 = 0;
    q21 = 12.92;
    q22 = -24;
    q23 = 0;
    
  tripleCubic(0,4,0,0,q00,q01,q10,q11,q20,q21,'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');
  tripleCubic(4,8,0,0,q01,q02,q11,q12,q21,q22,'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');
  tripleCubic(8,12,0,0,q02,q03,q12,q13,q22,q23,'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');
  tripleCubic(12,16,0,0,q03,q01,q13,q11,q23,q21,'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');

  while 1
      tic
      packet = zeros(15, 1, 'single');
      
      % create timestamp
      elapsed = etime(clock , timer);

      trajectoryj0 = csvread('trajectoryj0.csv');
      trajectoryj1 = csvread('trajectoryj1.csv');
      trajectoryj2 = csvread('trajectoryj2.csv');
      
      % ANGLES TO SEND
      % trajectory triangle (step 8-9)
      packet(1) = trajectoryj0(index, 4)* (4095/360);
      packet(4) = trajectoryj1(index, 4)* (4095/360); 
      packet(7) = trajectoryj2(index, 4)* (4095/360); 
      
      % Send packet to the server and get the response      
      % pp.write sends a 15 float packet to the micro controller
      pp.write(SERV_ID, packet); 
       
      pause(0.003); % Minimum amount of time required between write and read
       
      % pp.read reads a returned 15 float backet from the nucleo.
      returnPacket = pp.read(SERV_ID); 
      toc

      if DEBUG
          disp('Sent Packet:');
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end

      for i = 1:15
          el = returnPacket(i);
          m(index,i)=el;          
      end
       
      % creates matrix for csv logging joing angles
      % 1st column is time stamp
      % 2nd column = J0, 3rd column = J1, 4th column = J2
      pos(index,1)= elapsed;
      pos(index,2)=m(index,1);
      pos(index,3)=m(index,4);
      pos(index,4)=m(index,7);
      
      arm_plot(index,pos);
      
      % increase row # in matrix
      index = index + 1;

      toc
      
      pause(0.1); %time needed to loop through

      % write to CSV
      csvwrite('status.csv',m);
      csvwrite('joint_pos_log.csv',pos);
      
      % after Xsec, exit the loop and shut down
      if elapsed > 50
        break;
      end
      
end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');

end

% grab all of the task space joint angles and tip positions
joint_ang_vel_log = csvread('logJATP.csv');

% convert TRAJECTORY joint angles to cartesian
    traj_ang_pos = zeros(200,7);
    
% for count = 1:200
%     j0_ang = trajectoryj0(count,4);
%     j1_ang = trajectoryj1(count,4);
%     j2_ang = trajectoryj2(count,4);
%     [~,traj_xyz] = fwkin3001(j0_ang,j1_ang,j2_ang);
%     traj_x = traj_xyz(1);
%     traj_y = traj_xyz(2);
%     traj_z = traj_xyz(3);
%     
%     if count < 50
%         traj_ang_pos(count,1) = trajectoryj0(count,1);
%     elseif count < 100
%         traj_ang_pos(count,1) = trajectoryj0(count,1)+4;
%     elseif count < 150
%         traj_ang_pos(count,1) = trajectoryj0(count,1)+8;
%     else
%         traj_ang_pos(count,1) = trajectoryj0(count,1)+12;
%     end
%     traj_ang_pos(count,2) = j0_ang;
%     traj_ang_pos(count,3) = j1_ang;
%     traj_ang_pos(count,4) = j2_ang;
%     traj_ang_pos(count,5) = traj_x;
%     traj_ang_pos(count,6) = traj_y;
%     traj_ang_pos(count,7) = traj_z;
%     
% end

plots_lab2(traj_ang_pos);

% Clear up memory upon termination
pp.shutdown()
toc
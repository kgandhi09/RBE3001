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
  STATUS_ID = 03;                
  DEBUG   = false;         
  packet = zeros(15, 1, 'single');
 
  %initialize matrices
  m = zeros(25,15);
  pos = zeros(300,4);
  vel = zeros(300,4);
  index = 1;
  
  timer = clock;

  delete logJATP.csv;
  delete trajectoryj0.csv;
  delete trajectoryj1.csv;
  delete trajectoryj2.csv;
  trajectoryj0 = zeros(20000,5);
  trajectoryj1 = zeros(20000,5);
  trajectoryj2 = zeros(20000,5);
  
  % Triangle Lab 3-4
  point1 = [175,0,-35];
  point2 = [232.34,99.2,-14.154];
  point3 = [150,25,80];
  
%   q1 = inkin(point1)*(180/pi);
%   q2 = inkin(point2)*(180/pi);
%   q3 = inkin(point3)*(180/pi);

  q1 = [-0.020714,-0.027235,-0.0038359]*(180/pi);
  q2 = [-0.022248,-0.026851,0.0042195]*(180/pi);
  q3 = [-0.0230,0.0610,1.60]*(180/pi);
  
  %function tripleCubic(t0,tf,v0,vf,q0,q0f,q1,q1f,q2,q2f,file0,file1,file2)
  
  tripleCubic(0,4,0,0,q1(1),q2(1),q1(2),q2(2),q1(3),q2(3),'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');
  tripleCubic(4,8,0,0,q2(1),q3(1),q2(2),q3(2),q2(3),q3(3),'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');
  tripleCubic(8,12,0,0,q3(1),q1(1),q3(2),q1(2),q3(3),q1(3),'trajectoryj0.csv','trajectoryj1.csv','trajectoryj2.csv');
  
  % Angles to Send
  trajectoryj0 = csvread('trajectoryj0.csv');
  trajectoryj1 = csvread('trajectoryj1.csv');
  trajectoryj2 = csvread('trajectoryj2.csv');
  
  [traj_length,~] = size(trajectoryj0);
  
  waypoint = 1;
  
  [x,z] = ginput(1)
  pause(5);
  
while 1
      tic
      packet = zeros(15, 1, 'single');
      
      % create timestamp
      elapsed = etime(clock , timer);
      
%       % time-based trajectory movement
%       while trajectoryj0(waypoint,1) < elapsed
%         waypoint = waypoint + 1;      
%         if waypoint > traj_length
%           break;
%         end
%       end
%       if waypoint > traj_length
%           break;
%       end
%       % cubic trajectory triangle
%       packet(1) = trajectoryj0(waypoint,4)* (4095/360);
%       packet(4) = trajectoryj1(waypoint,4)* (4095/360); 
%       packet(7) = trajectoryj2(waypoint,4)* (4095/360); 

      % Send packet to the server and get the response      
      % pp.write sends a 15 float packet to the micro controller
      pp.write(SERV_ID, packet); 
      pause(0.003); % Minimum amount of time required between write and read
      
      % pp.read reads a returned 15 float backet from the nucleo.
      returnPacket = pp.read(SERV_ID); 

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
       
      % creates matrix for csv logging joint angles
      % 1st column is time stamp
      % 2nd column = J0, 
      % 3rd column = J1,
      % 4th column = J2
      pos(index,1)= elapsed;
      pos(index,2)=m(index,1);
      pos(index,3)=m(index,4);
      pos(index,4)=m(index,7);
      
      vel(index,1)= elapsed;
      vel(index,2)=m(index,2);
      vel(index,3)=m(index,5);
      vel(index,4)=m(index,8);
 
      %arm_plot(index,pos,vel);
      
      
      % increase row # in matrix
      index = index + 1;

      % write to CSV
      % csvwrite('status.csv',m);
      csvwrite('joint_pos_log.csv',pos);
      
      
      numinkin([0,0,0],[x*280,0,z*400]);
      
      % after Xsec, exit the loop and shut down
      if elapsed > 20
        break;
      end
      
end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');

end

% figures for LAB 2
% grab all of the joint angles and tip positions
joint_ang_vel_log = csvread('logJATP.csv');
% plots_lab2(joint_ang_vel_log);

disp(elapsed);
% Clear up memory upon termination
pp.shutdown()
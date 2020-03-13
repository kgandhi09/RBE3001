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

  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
%   viaPts = [0, -400, 400, -400, 400, 0];
 
  %initialize matrices
  m = zeros(25,15);
  n = zeros(25,15);
  l = zeros(25,15);
  
  pos = zeros(25,4);
  index = 1;
  prev_time = 0;
  
timer = clock;

while 1
      tic
      packet = zeros(15, 1, 'single');
      
      %positions for j0,j1,andj2, respectively
      packet(1) = 200;
      packet(4) = 225;
      packet(7) = -20;

      % Send packet to the server and get the response      
      %pp.write sends a 15 float packet to the micro controller
       pp.write(SERV_ID, packet); 
       
       pause(0.003); % Minimum amount of time required between write and read
       
       %pp.read reads a returned 15 float backet from the nucleo.
       returnPacket = pp.read(SERV_ID); 
      toc

      if DEBUG
          disp('Sent Packet:');
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end
      
      % create timestamp
      elapsed = etime(clock , timer);

      for i = 1:15
          el = returnPacket(i);
          m(index,i)=el;
      %   n(index,i)=el;
      %   l(index,i)=el;
          
%         used for creating multiple plots, uncomment as needed
         figure(1);
         plot(m(:,1));
         title('Joint 0');
      %   figure(2);
      %   plot(n(:,4));
      %   title('Joint 1');
      %   figure(3);
      %   plot(l(:,7));
      %   title('Joint 2');
          
      end
       
%       creates matrix for csv logging joing angles
%       1st column is time stamp
%       2nd column = J0, 3rd column = J1, 4th column = J2
      pos(index,1)= elapsed;
      pos(index,2)=m(index,1);
      pos(index,3)=m(index,4);
      pos(index,4)=m(index,7);
      
%     increase row # in matrix
      index = index + 1;

      toc
      
      pause(0.1); %time needed to loop through

%     create the matrices
      csvwrite('status.csv',m);
      csvwrite('joint_pos_log.csv',pos);
      
%     after 10sec, exit the loop and shut down
      if elapsed > 10
        break;
      end
      
      
end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');

end

% Clear up memory upon termination
pp.shutdown()
toc

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
    STATUS_ID = 03;            % we will be talking to server ID 01 on the Nucleo
    
    DEBUG   = true;            % disables debug prints
    
    % Instantiate a packet - the following instruction allocates 64
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes.
    packet = zeros(15, 1, 'single');
    
    %initialize matrices
    m = zeros(25,15);
    pos = zeros(300,4);
    index = 1;
    timer = clock;
    
    delete logJATP.csv;
    delete linearTrajectory.csv;
    delete trajectoryX.csv;
    delete trajectoryY.csv;
    delete trajectoryZ.csv;
    trajectoryj0 = zeros(20000,5);
    trajectoryj1 = zeros(20000,5);
    trajectoryj2 = zeros(20000,5);
    
    point1 = [175,0,-35];
    point2 = [232.34,99.2,-14.154];
    point3 = [150,25,80];
    
    quinticTrajectory(0,4,0,0,0,0,point1(1),point2(1),point1(2),point2(2),point1(3),point2(3),...
                      'trajectoryX.csv','trajectoryY.csv','trajectoryZ.csv');
    quinticTrajectory(4,8,0,0,0,0,point2(1),point3(1),point2(2),point3(2),point2(3),point3(3),...
                      'trajectoryX.csv','trajectoryY.csv','trajectoryZ.csv');
    quinticTrajectory(8,12,0,0,0,0,point3(1),point1(1),point3(2),point1(2),point3(3),point1(3),...
                      'trajectoryX.csv','trajectoryY.csv','trajectoryZ.csv');
                  
    %writes X Y Z positions into 'linearTrajectory.csv'
    linearTrajectory(point1,point2,'linearTrajectory.csv');
    linearTrajectory(point2,point3,'linearTrajectory.csv');
    linearTrajectory(point3,point1,'linearTrajectory.csv');

    trajectoryX = csvread('trajectoryX.csv');
    trajectoryY = csvread('trajectoryY.csv');
    trajectoryZ = csvread('trajectoryZ.csv');

    merged = [trajectoryX trajectoryY trajectoryZ]; % X Y Z MATRIX
    waypoint = 1;
                  
    j=zeros(1,3);
    packet = zeros(15, 1, 'single');
    t = zeros(2,1);
    
    while 1
        tic
        
        % create timestamp
        elapsed = etime(clock , timer);
       
        % LAB 3 PART 8
        if merged(waypoint,1) < elapsed
            waypoint = waypoint + 1;
        end
        if waypoint > size(merged,1)
            break;
        end
        joints = inkin([merged(waypoint,2), merged(waypoint,9), merged(waypoint,16)]); % JOINT 1, JOINT2, JOINT3 vertically
        disp(waypoint);
        % Angles to Send in RADIANS
        packet(1) = joints(1)* (4095/(2*pi));  % joint 0
        packet(4) = joints(2)* (4095/(2*pi));  % joint 1
        packet(7) = joints(3)* (4095/(2*pi));  % joint 2
        
        % Send packet to the server and get the response
        % pp.write sends a 15 float packet to the micro controller
        pp.write(SERV_ID, packet);
        pause (0.003);
        
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
        % 4th column = J2,
        pos(index,1)= elapsed;
        pos(index,2)=m(index,1);
        pos(index,3)=m(index,4);
        pos(index,4)=m(index,7);
        
        j0 = pos(index,2);
        j1 = pos(index,3);
        j2 = pos(index,4);
        % convert angles to rad for LAB 3 PART 4
        j0_rad = j0 * (2*pi)/4095; 
        j1_rad = j1 * (2*pi)/4095;
        j2_rad = j2 * (2*pi)/4095;
        
        % COMMENT out for LAB 3 PART 4 TO OPTIMIZE COMMUNICATION
        arm_plot(index,pos);
        
        % increase row # in matrix
        index = index + 1;     
        
        % write to CSV
        csvwrite('status.csv',m);
        csvwrite('joint_pos_log.csv',pos);
        
        if elapsed > 50
            break;
        end
        
        % LAB 2 PART 4
        gap = 500;
        if index > gap
            t(index-gap) = toc;
        end
   
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
    
end

% LAB 3 PART 4
histogram(t,500);
xlim([0,0.01]);
ylim([0,50]);
average = mean(t);
min = min(t);
max = max(t(2:end,1));
SD = std(t);
disp("average:            " + average);
disp("minimum:            " + min);
disp("maximum:            " + max);
disp("standard deviation: " + SD);

% figures for LAB 3
% grab all of the joint angles and tip positions
joint_ang_vel_log = csvread('logJATP.csv');
plots_lab2(joint_ang_vel_log);

% Clear up memory upon termination
disp(elapsed)
pp.shutdown()
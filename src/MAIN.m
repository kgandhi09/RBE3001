%% MAIN
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

cam = webcam();
camParameters;

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

try
    SERV_ID = 01;
    STATUS_ID = 03;            % we will be talking to server ID 01 on the Nucleo
    GRIPPER_SERV_ID = 5;
    
    DEBUG   = false;            % disables debug prints
    
    % Instantiate a packet - the following instruction allocates 64
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes.
    packet = zeros(15, 1, 'single');
    
    %initialize matrices
    m = zeros(25,15);
    pos = zeros(300,4);
    
    timer = clock;
    
    delete logJATP.csv;
    delete linearTrajectory.csv;
    delete traj0.csv;
    delete traj1.csv;
    delete traj2.csv;
    traj0 = zeros(20000,5);
    traj1 = zeros(20000,5);
    traj2 = zeros(20000,5);
    
    startingJoint = [-1.2858,-0.2608,0.1841];
    
    j=zeros(1,3);
    
    % initialize gripper packet
    grip_packet = zeros(15, 1, 'single');
    
    while 1
        tic
        % create timestamp
        elapsed = etime(clock , timer);
        
        delete traj0.csv;
        delete traj1.csv;
        delete traj2.csv;
        
        [coords, colors, areas] = findObjs(snapshot(cam), cameraParams);
        for ind = 1:size(areas)
            if(ind > size(coords, 1))
                disp("Coords!")
            end
            if(ind > size(colors, 1))
                disp("Colors!")
            end
            if(ind > size(areas, 1))
                disp(ind)
                disp(areas)
            end
            pickAndPlace(coords(ind,:),startingJoint,colors(ind),areas(ind))
        end
        
        %if no objects are detected, break
        if size(colors) == 0
            break;
        end
        %read in trajectories to matrix
        traj0 = csvread('traj0.csv');
        traj1 = csvread('traj1.csv');
        traj2 = csvread('traj2.csv');
        index = 1;
        
        while 1
            
            % avoid going through the matrix more than needed
            if index > size(traj0)
                break;
            end
                
            % what to send to the robots
            if traj0(index,1) == 99999
                pause(0.5);
            elseif traj0(index,1) == 99998 % gripper open
                grip_packet(1) = 1;
                pp.write(GRIPPER_SERV_ID, grip_packet);
                pause(1);
            elseif traj0(index,1) == 99997 % gripper close
                grip_packet(1) = 0;
                pp.write(GRIPPER_SERV_ID, grip_packet);
                pause(1);
                
            else
                % move in trajectory path
                packet(1) = traj0(index,4)* (4095/(2*pi));
                packet(4) = traj1(index,4)* (4095/(2*pi));
                packet(7) = traj2(index,4)* (4095/(2*pi));
                packet(10) = 1;
                
                % Send packet to the server and get the response
                % pp.write sends a 15 float packet to the micro controller
                pp.write(SERV_ID, packet);
                pause(0.043);
            end
            
            % pp.read reads a returned 15 float backet from the nucleo.
            returnPacket = pp.read(SERV_ID);pause (0.003);
            
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
            
            % increase row # in matrix
            index = index + 1;
            
        end
        
        coords = [];
        colors = [];
        areas = [];
        
        % times out after 2min 
        if elapsed > 120
            break;   
        end
        
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown')
end

% Clear up memory upon termination
disp(elapsed)
pp.shutdown()
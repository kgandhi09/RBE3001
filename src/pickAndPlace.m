function pickAndPlace(centroidXY,joints,color,size)
file0 = 'traj0.csv';
file1 = 'traj1.csv';
file2 = 'traj2.csv';

%% Sorting Process

% BLUE  = 1 
% GREEN  = 2
% YELLOW = 3
% determines the X position for a color 
if color == 1
    endX = 25;
elseif color == 2
    endX = 100;
elseif color == 3
    endX = 175;
else
    disp('Color not Valid!!');
end

% determines the Y position of a size (big = left & small = right of robot)
if size > 3000
    endY = 225;
else
    endY = -225;
end

[~,endEffector] = fwkin3001(joints(1),joints(2),joints(3));   %returns j2 and ee

%equation from 0,0 to centroidX, centroidY

%% Attempt to Manually Offset the Position
modCentroidXY = [0,0];

xDisp = 25;
yDisp = 25;

% disp(centroidXY)
if centroidXY(2) > 0
    modCentroidXY = [centroidXY(1)+xDisp,centroidXY(2)+yDisp];
else 
    modCentroidXY = [centroidXY(1)+xDisp,centroidXY(2)-yDisp];
end

%% Cubic Trajectory Generation
startXYZ = endEffector.'; %starting XYZ
aboveStartXYZ = [endEffector(1),endEffector(2), 50]; %3 inches above the ground 
aboveEndXYZ = [modCentroidXY(1), modCentroidXY(2), 50];
disp(aboveEndXYZ)
endXYZ = [modCentroidXY(1), modCentroidXY(2), -25]; %-45 + 31.75 = 13 %ending XYZ
aboveSortXYZ = [endX, endY, 31]; %at the determined sort location based off of above if statements
sortXYZ = [endX, endY, -13];

v0 = 0;
vf = 0;

x = [startXYZ(1), aboveStartXYZ(1), aboveEndXYZ(1), endXYZ(1), aboveEndXYZ(1), aboveSortXYZ(1), sortXYZ(1)]; 
y = [startXYZ(2), aboveStartXYZ(2), aboveEndXYZ(2), endXYZ(2), aboveEndXYZ(2), aboveSortXYZ(2), sortXYZ(2)];
z = [startXYZ(3), aboveStartXYZ(3), aboveEndXYZ(3), endXYZ(3), aboveEndXYZ(3), aboveSortXYZ(3), sortXYZ(3)];

%-> Output [Joint1 Value, Joint2 Value, Joint3 Value]c
jp1 = inkin([x(1), y(1), z(1)]);   %Joint Values for starting point
jp2 = inkin([x(2), y(2), z(2)]);   %Joint Values for above start point (exit)
jp3 = inkin([x(3), y(3), z(3)]);   %Joint Values for above end point (approach)
jp4 = inkin([x(4), y(4), z(4)]);   %Joint Values for end point (approach) 
jp5 = inkin([x(5), y(5), z(5)]);   %Joint Values for above end point (exit)
jp6 = inkin([x(6), y(6), z(6)]);   %Joint Values above sort point (approach/exit)
jp7 = inkin([x(7), y(7), z(7)]);   %Joint Values at sort point

stopCommand = [99999,99999,99999,99999,99999];
gOpen = [99998,99998,99998,99998,99998];
gClose = [99997,99997,99997,99997,99997];

dlmwrite(file0,stopCommand,'-append')
dlmwrite(file0,gOpen,'-append')
dlmwrite(file1,stopCommand,'-append')
dlmwrite(file1,gOpen,'-append')
dlmwrite(file2,stopCommand,'-append')
dlmwrite(file2,gOpen,'-append')

tripleCubic(0,5,v0,vf,jp1(1),jp2(1),jp1(2),jp2(2),jp1(3),jp2(3),file0,file1,file2); % jp1 -> jp2
tripleCubic(5,10,v0,vf,jp2(1),jp3(1),jp2(2),jp3(2),jp2(3),jp3(3),file0,file1,file2); % jp2 -> jp3
tripleCubic(10,15,v0,vf,jp3(1),jp4(1),jp3(2),jp4(2),jp3(3),jp4(3),file0,file1,file2); % jp3 -> jp4

dlmwrite(file0,stopCommand,'-append')
dlmwrite(file0,gClose,'-append')
dlmwrite(file1,stopCommand,'-append')
dlmwrite(file1,gClose,'-append')
dlmwrite(file2,stopCommand,'-append')
dlmwrite(file2,gClose,'-append')

tripleCubic(20,25,v0,vf,jp4(1),jp5(1),jp4(2),jp5(2),jp4(3),jp5(3),file0,file1,file2); % jp4 -> jp5
tripleCubic(25,30,v0,vf,jp5(1),jp6(1),jp5(2),jp6(2),jp5(3),jp6(3),file0,file1,file2); % jp5 -> jp6
tripleCubic(30,35,v0,vf,jp6(1),jp7(1),jp6(2),jp7(2),jp6(3),jp7(3),file0,file1,file2); % jp5 -> jp6

dlmwrite(file0,stopCommand,'-append')
dlmwrite(file0,gOpen,'-append')
dlmwrite(file1,stopCommand,'-append')
dlmwrite(file1,gOpen,'-append')
dlmwrite(file2,stopCommand,'-append')
dlmwrite(file2,gOpen,'-append')
<<<<<<< HEAD

tripleCubic(35,40,v0,vf,jp7(1),jp6(1),jp7(2),jp6(2),jp7(3),jp6(3),file0,file1,file2); % jp5 -> jp6
=======
%go up after dropping off object
tripleCubic(35,40,v0,vf,jp7(1),jp6(1),jp7(2),jp6(2),jp7(3),jp6(3),file0,file1,file2); % jp6 -> jp7
>>>>>>> 1b2a55c53a45d2145a3d88fc2ebc53b98f8fe9ab
end
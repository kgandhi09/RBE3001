function move2pick(centroidXY,joints,color,size)
file0 = 'traj0.csv';
file1 = 'traj1.csv';
file2 = 'traj2.csv';

[~,endEffector] = fwkin3001(joints(1),joints(2),joints(3));   %returns j2 and ee

startXYZ = endEffector.'; %starting XYZ
aboveStartXYZ = [endEffector(1),endEffector(2), 31]; %3 inches above the ground 
aboveEndXYZ = [centroidXY(1), centroidXY(2), 31];
endXYZ = [centroidXY(1), centroidXY(2), -13]; %-45 + 31.75 = 13 %ending XYZ

v0 = 0;
vf = 0;
a0 = 0;
af = 0;

x = [startXYZ(1), aboveStartXYZ(1), aboveEndXYZ(1), endXYZ(1)]; 
y = [startXYZ(2), aboveStartXYZ(2), aboveEndXYZ(2), endXYZ(2)];
z = [startXYZ(3), aboveStartXYZ(3), aboveEndXYZ(3), endXYZ(3)];


disp(x)
disp(y)
disp(z)

%-> Output [Joint1 Value, Joint2 Value, Joint3 Value]c
jp1 = inkin([x(1), y(1), z(1)]);   %Joint Values for starting point
jp2 = inkin([x(2), y(2), z(2)]);   %Joint Values for above start point (exit)
jp3 = inkin([x(3), y(3), z(3)]);   %Joint Values for above end point (approach) 
jp4 = inkin([x(4), y(4), z(4)]);   %Joint Values for end point (approach) 

tripleCubic(0,5,v0,vf,jp1(1),jp2(1),jp1(2),jp2(2),jp1(3),jp2(3),file0,file1,file2); % jp1 -> jp2
tripleCubic(5,10,v0,vf,jp2(1),jp3(1),jp2(2),jp3(2),jp2(3),jp3(3),file0,file1,file2); % jp2 -> jp3
tripleCubic(10,15,v0,vf,jp3(1),jp4(1),jp3(2),jp4(2),jp3(3),jp4(3),file0,file1,file2); % jp3 -> jp4

end
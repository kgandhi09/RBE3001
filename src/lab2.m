% RBE3001 Team 7 Lab 2 MATLAB Code

% FK of the RBE3001 Arm:
% Create a MATLAB function fwkin3001 that takes the three joint angles as inputs, and returns a 3x1
% vector of the tip position, p, with respect to the base frame F0 (refer to the “Arm Configuration”
% figure at the end of this document).

% disp(fwkin3001(0, 0.75*pi, 0.5*pi));

length1 = 135;
length2 = 175;
length3 = 169.28;

joint0 = 0;
joint1 = 90;
joint2 = 90;

point0 = [0,0,0];
point1 = [0,0,135];
point2 = [fwkinPoint2(joint0,joint1)];
point3 = [fwkin3001(joint0,joint1,joint2)];

matrixX = [point0(1,1),point1(1,1),point2(1,1),point3(1,1)];
matrixY = [point0(1,2),point1(1,2),point2(1,2),point3(1,2)];
matrixZ = [point0(1,3),point1(1,3),point2(1,3),point3(1,3)];

plot = plot3(matrixX,matrixY,matrixZ);

d = [135, 0, 0];  
a = [0, 175, 169.28]; 
alpha = [-pi/2, 0, 0];

function p = fwkin3001(j0,j1,j2) %end effector
    T_01 = tdh(j0, d(1), a(1), alpha(1));
    T_12 = tdh(j1, d(2), a(2), alpha(2));
    T_23 = tdh(j2, d(3), a(3), alpha(3));
    
    T_03 = T_01 * T_12 * T_23;    
    
    p = [T_03(1,4); T_03(2,4); T_03(3,4)];
end

function p = fwkinPoint2(j0,j1) 
    T_01 = tdh(j0, d(1), a(1), alpha(1));
    T_12 = tdh(j1, d(2), a(2), alpha(2));
    
    T_03 = T_01 * T_12;     
    
    p = [T_03(1,4); T_03(2,4); T_03(3,4)];
end

% DH Transformation
function T = tdh(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end





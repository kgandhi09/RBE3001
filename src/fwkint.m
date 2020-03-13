% same as fwkin3001, but returns the transformation matrices for all joints
function [T_01,T_02,T_03] = fwkint(j0,j1,j2) 
    %DH Parameters
    a = [0, 175, 169.28]; 
    alpha = [-pi/2, 0, 0];
    d = [135, 0, 0];  
    
    T_01 = tdh(j0, d(1), a(1), alpha(1));
    T_12 = tdh(j1, d(2), a(2), alpha(2));
    T_23 = tdh(j2 +(pi/2), d(3), a(3), alpha(3));
    
    T_02 = T_01 * T_12;
    p2 = [T_02(1,4); T_02(2,4); T_02(3,4)];
    
    T_03 = T_01 * T_12 * T_23;   
    p3 = [T_03(1,4); T_03(2,4); T_03(3,4)];
end

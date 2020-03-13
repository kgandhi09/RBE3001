% function jacob0 inputs q (in radians) which is a matrices of joint angles 
% from 0 to 2 and outputs 6X3 jacobian matrix

function j = jacob0(q)
    
    %joint angles of j0, j1, j2 from q
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);
    
    %link lengths
    l1 = 135;
    l2 = 175;
    l3 = 169.28;

    % Perform calculations via the Cross Product Method
    T_00 = eye(4);
    [T_01,T_02,T_03] = fwkint(t1,t2,t3);
    
    z0 = T_00(1:3,3);
    z1 = T_01(1:3,3);
    z2 = T_02(1:3,3);
    
    p0 = T_00(1:3,4);
    p1 = T_01(1:3,4);
    p2 = T_02(1:3,4);
    pe = T_03(1:3,4);
    
    % jacobian
    j = [cross(z0, (pe-p0)) cross(z1, (pe-p1)) cross(z2, (pe-p2));...
         z0 z1 z2];
end
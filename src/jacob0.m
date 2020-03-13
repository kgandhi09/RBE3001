%function jacob0 inputs q which is a matrices of joint angles from 0 to 2
%and outputs 6X3 jacobian matrix
% inputs are in radians
function j = jacob0(q)
    
    %joint angles of j0, j1, j2 from q
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);
    
    %link lengths
    l1 = 135;
    l2 = 175;
    l3 = 169.28;
    
    %positional Jp
%     PX_THETA0 = - l2*cos(t2)*sin(t1) - l1*cos(t2)*sin(t1)*sin(t3) - l3*cos(t3)*sin(t1)*sin(t2);
%     PX_THETA1 = l3*cos(t1)*cos(t2)*cos(t3) - l2*cos(t1)*sin(t2) - l1*cos(t1)*sin(t2)*sin(t3);
%     PX_THETA2 = l1*cos(t1)*cos(t2)*cos(t3) - l3*cos(t1)*sin(t2)*sin(t3);
%     PY_THETA0 = l2*cos(t1)*cos(t2) + l3*cos(t1)*cos(t2)*sin(t3) + l3*cos(t1)*cos(t3)*sin(t2);
%     PY_THETA1 = l3*cos(t2)*cos(t3)*sin(t1) - l2*sin(t1)*sin(t2) - l3*sin(t1)*sin(t2)*sin(t3);
%     PY_THETA2 = l3*cos(t2)*cos(t3)*sin(t1) - l3*sin(t1)*sin(t2)*sin(t3);
%     PZ_THETA0 = 0;
%     PZ_THETA1 = l2*cos(t2) + l1*cos(t2)*sin(t3) + l3*cos(t3)*sin(t2);
%     PZ_THETA2 = l1*cos(t3)*sin(t2) + l3*cos(t2)*sin(t3);
    
    % negatives for theta0 because direction was reversed on the arm plot
%     j = [PX_THETA0, PX_THETA1, PX_THETA2;
%          PY_THETA0, PY_THETA1, PY_THETA2;
%          PZ_THETA0, PZ_THETA1, PZ_THETA2;
%          0,         sin(t1),    sin(t1);
%          0,         -cos(t1),  -cos(t1);
%          -1,   0,         0];

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
%function jacob0 inputs q which is a matrices of joint angles from 0 to 2
%and outputs 6X3 jacobian matrix
% inputs are in radians
function j = new_jacobion(q)
    syms t1 t2 t3 l1 l2 l3
%     l1 = 135;
%     l2 = 175;
%     l3 = 169.28
    px = l1*cos(t1)*cos(t2)*sin(t3) + l3*cos(t1)*sin(t2)*cos(t3) + l2*cos(t1)*cos(t2);
    py = l3*sin(t1)*cos(t2)*sin(t3) + l3*sin(t1)*sin(t2)*cos(t3) + l2*sin(t1)*cos(t2);
    pz = l1*sin(t2)*sin(t3) - l3*cos(t2)*cos(t3) + l2*sin(t2) + l1;
    j = jacobian([px, py, pz], [t1,t2,t3]);
    
    
end
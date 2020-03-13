
% takes as input a 3x1 (x,y,z) tip position in the task space of the robot
% returns a set of corresponding joint angles that would put the robot’s end effector  
function joints = inkin(xyz)
    % grab point entry
    px = xyz(1);
    py = -xyz(2);
    pz = xyz(3);
    
    if px > 280 | px < 103 | abs(py) > (140) | pz < -45
        error('Not within workspace');
    end
    
    % Link Lengths (mm)
    L = [135, 175, 169.28];

    d =  pz - L(1);
    h = sqrt(px^2 + py^2);
    c = sqrt(h^2 + d^2);
    
    theta1 = atan2(py, px);
    
    beta = acos((c^2 + L(2)^2 - L(3)^2)/(2*c*L(2)));
    phi = atan2(d,h);
    theta2 = beta + phi;
    
    alpha = acos((L(3)^2 + L(2)^2 - c^2)/(2*L(3)*L(2)));
    theta3 = alpha - pi/2;
    
    joints = [theta1, theta2, theta3];

end
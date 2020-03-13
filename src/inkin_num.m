% Jacobian-based solution of the inverse kinematics problem

function sol = inkin_num(q0,pd)

%start at q0
%end at pd (position destination (XYZ)) task space
%the entire

timer = clock;
qi = q0;
while 1
    jp_qi = jacob0(qi);
    jp_qi = jp_qi(1:3,:);
    %del q    task to point * point - point 
    delta_q = pinv(jp_qi)*(pd-fwkin3001(qi(1), qi(2), qi(3)));
    if norm(pd -fwkin3001(qi(1), qi(2), qi(3))) > 0.5
        pause(0.25)
        qi = qi + delta_q;
        disp(qi.');
    else
        break;
    end
    
    elapsed = etime(clock , timer);
    if elapsed > 20
        sol = fwkin3001(qi(1), qi(2), qi(3));
        break;
    end
    
end
end

%q0 = initial position (JOINT SPACE)
%pd = final position   (TASK SPACE)
function numinkin(q0,pd) 
qi = q0;  %TASK SPACE
epsilon = 0.1; %TOLERANCE
fwkin_qi = fwkin3001(qi(1), qi(2), qi(3));
magnitude1 = sqrt((pd(1)^2 - fwkin_qi(1)^2) + (pd(2)^2 - fwkin_qi(2)^2) + (pd(3)^2 - fwkin_qi(3)^2)); %distance equation in 3D

while magnitude1 > epsilon
    inverse_jacobian = pinv(jacob0(qi)); % j^-1(qi) inverse jacobian
    magnitude1 = sqrt((pd(1)^2 - fwkin_qi(1)^2) + (pd(2)^2 - fwkin_qi(2)^2) + (pd(3)^2 - fwkin_qi(3)^2)); %distance equation in 3D
    delta_q = inverse_jacobian * magnitude1; %calculating delta_q
    qi = qi + delta_q;
end
end

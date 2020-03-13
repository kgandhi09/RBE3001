function qi = numinkin(q0,pd)
qi = q0;  %TASK SPACE
epsilon = 1; %TOLERANCE
[~, fwkin_qi] = fwkin3001(qi(1), qi(2), qi(3));
mag = norm(pd - fwkin_qi);

timer = clock;

while mag > epsilon
    jp = jacob0(qi);
    jp = jp(1:3,1:3);
    inverse_jacobian = pinv(jp); % j^-1(qi) inverse jacobian
    [~, fwkin_qi] = fwkin3001(qi(1), qi(2), qi(3));
    mag = norm(pd - fwkin_qi);
    delta_q = inverse_jacobian * (pd - fwkin_qi); % calculating delta_q
    stick_figure(-qi);
    qi = qi + delta_q;
    disp(mag);
    pause(0.1);
    elapsed = etime(clock, timer);
    if elapsed > 10
        break;
    end
end
disp(fwkin_qi);
disp("done")
end
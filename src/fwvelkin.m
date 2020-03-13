function p_vel = fwvelkin(q, qvels)
    J = jacob0(q);
    p_vel = J*qvels.';
end
%Write a MATLAB function that solves for a cubic polynomial trajectory. It should take in: desired start
%and end times t 0 and t f (in seconds), start and end velocities (in degree/sec), and start and end joint
%positions (in degrees). It should output a 4x1 array containing the coefficients a i , i = 0,1,2,3 of the
%polynomial.

% csv outputs columns time,q0,v0,qf,vf

function tripleCubic(t0,tf,v0,vf,q0,q0f,q1,q1f,q2,q2f,file0,file1,file2)
    cubicTrajectory(t0,tf,v0,vf,q0,q0f,file0);
    cubicTrajectory(t0,tf,v0,vf,q1,q1f,file1);
    cubicTrajectory(t0,tf,v0,vf,q2,q2f,file2);
end

function cubicTrajectory(t0,tf,v0,vf,q0,qf,filename)
    rez = 25;
    a= outputA(t0,tf,v0,vf,q0,qf);
    timeElapsed = tf-t0;
    timeInterval = timeElapsed/rez;
    M = zeros(1,5);
    index = t0;
    while 1
        b = outputB(a,index,index+timeInterval);
        M = [index,b(1),b(2),b(3),b(4)];
        dlmwrite(filename,M,'-append');
        index = index + timeInterval;
        if index > tf
            break
        end
    end
end

%ouputs the a matrix
function a = outputA(t0,tf,v0,vf,q0,qf)
    cubic = [1,t0,(t0)^2,(t0)^3;
             0,1,2*t0,3*(t0)^2;
             1,(tf),(tf)^2,(tf)^3;
             0,1,2*(tf),3*(tf)^2];
    invcubic = inv(cubic);
    polynomial = [q0;v0;qf;vf];
    a = invcubic * polynomial;
end

%outputs v0,q0,qf, vf
function b = outputB(a,t0,tf)
    cubic = [1,t0,(t0)^2,(t0)^3;
             0,1,2*t0,3*(t0)^2;
             1,(tf),(tf)^2,(tf)^3;
             0,1,2*(tf),3*(tf)^2];
    b = cubic*a;
end
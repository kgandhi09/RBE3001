
function quinticTrajectory(t0,tf,v0,vf,a0,af,x0,xf,y0,yf,z0,zf,file0,file1,file2)
    quinticPolynomial(t0,tf,v0,vf,x0,xf,a0,af,file0);
    quinticPolynomial(t0,tf,v0,vf,y0,yf,a0,af,file1);
    quinticPolynomial(t0,tf,v0,vf,z0,zf,a0,af,file2);
end

%Function that takes in real time start and end time values
%initial and final velocities
%incitial and final accelerations
%fiLename to log xyz values
function quinticPolynomial(t0,tf,v0,vf,xyz0,xyzf,a0,af,filename)
    rez = 10;
    a= outputA(t0,tf,v0,vf,xyz0,xyzf,a0,af);
    timeElapsed = tf-t0;
    timeInterval = timeElapsed/rez;
    M = zeros(1,7);
    index = t0;
    while 1
        b = outputB(a,index,index+timeInterval);
        M = [index,b(1),b(2),b(3),b(4),b(5),b(6)];
        dlmwrite(filename,M,'-append');
        index = index + timeInterval;
        if index > tf
            break
        end
    end
end


%MATLAB function that solves for a quintic polynomial trajectory. It should take in: desired start
%and end times t 0 and t f (in seconds), start and end velocities (in degree/sec), 
%start and end accelerations and start and end x,y and z positions(in degrees).
%Outputs a 6x1 matrix with coefficients of quintic polynomial
function a = outputA(t0,tf,v0,vf,xyz0,xyzf,a0,af)

    quintic = [1,t0,(t0)^2,(t0)^3,(t0)^4,(t0)^5;
             0,1,2*t0,3*(t0)^2,4*(t0)^3,5*(t0)^4;
             0,0,2,6*(t0),12*(t0)^2,20*(t0)^3;
             1,(tf),(tf)^2,(tf)^3,(tf)^4,(tf)^5;
             0,1,2*(tf),3*(tf)^2,4*(tf)^3,5*(tf)^4;
             0,0,2,6*(tf),12*(tf)^2,20*(tf)^3];
    invquintic = inv(quintic);
    polynomial = [xyz0;v0;a0;xyzf;vf;af];
    a = invquintic * polynomial;
end

%MATLAB function that solves for a quintic polynomial trajectory. It should take in: desired start
%and end times t 0 and t f (in seconds),
%matrix of coefficients of quintic polynomial (a)
%Outputs a 6x1 matrix - [q0,v0,a0,qf,vf,af]
function b = outputB(a,t0,tf)
    quintic = [1,t0,(t0)^2,(t0)^3,(t0)^4,(t0)^5;
             0,1,2*t0,3*(t0)^2,4*(t0)^3,5*(t0)^4;
             0,0,2,6*(t0),12*(t0)^2,20*(t0)^3;
             1,(tf),(tf)^2,(tf)^3,(tf)^4,(tf)^5;
             0,1,2*(tf),3*(tf)^2,4*(tf)^3,5*(tf)^4;
             0,0,2,6*(tf),12*(tf)^2,20*(tf)^3];
    b = quintic*a;
end
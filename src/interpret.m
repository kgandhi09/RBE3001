
%v0 q0 vf qf


function interpret(file1,file2,file3)
   
    trajectory0 = readFile(file1);
    trajectory1 = readFile(file1);
    trajectory2 = readFile(file1);
    
    time = trajectory0(:,1);
    traj0vel = trajectory0(:,4);
    traj1vel = trajectory1(:,4);
    traj2vel = trajectory2(:,4);
    
    plot(time, traj0vel);
    hold on 
    plot(time, traj1vel);
    plot(time, traj2vel);
    hold off
    
end


function file = readFile(filename)
    file = csvread(filename);
end
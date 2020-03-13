%linear point to point (lab 3 part 6): interpolated motion in task space

%taking in X0,Y0,Z0 (point0), X1,Y1,Z1 (point1) and creating a trajectory
%outputs a 20(equal to rez) by 3 matrix

%output to a csv file
function linearTrajectory(point0,point1,filename)
    rez = 10; %resolution 
    line = zeros(2,3);
    for index = 1:rez
        lineEquation = point0 + (index/rez)*(point1-point0);
        line(index,1) = lineEquation(1);
        line(index,2) = lineEquation(2);
        line(index,3) = lineEquation(3);
    end
    dlmwrite(filename,line,'-append');
end




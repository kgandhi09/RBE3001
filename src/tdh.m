% DH Transformation Function
function T = tdh(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

%SYMBOLIC
% [cos(sym(theta)), -sin(sym(theta))*cos(sym(alpha)), sin(sym(theta))*sin(sym(alpha)), a*cos(sym(theta));
%     sin(sym(theta)), cos(sym(theta))*cos(sym(alpha)), -cos(sym(theta))*sin(sym(alpha)), a*sin(sym(theta));
%     0, sin(sym(alpha)), cos(sym(alpha)), d;
%     0, 0, 0, 1];
     
     
    


% takes in camera parameters and points in the camera frame and converts
% them into the robot's frame
function robot_coord = camera2robot(camParams, camPoints)

% initialize transformation matrices
T_checker_robot = [-1.0000   0         0     275.8; ... % 175 + 52.8 +48
                   0    1.0000         0  113.6000; ... % 101.6 + 12
                   0         0   -1.0000         0; ...
                   0         0         0    1.0000];
          
T_cam_checker = [0.0402   -0.8241    0.5650  118.0120; ...
                 0.9978    0.0032   -0.0663   93.2302; ...
                 0.0528    0.5665    0.8224  273.9632; ...
                 0         0         0       1.0000];

% T_cam_checker = [0.0320   -0.8306    0.5560  122.7929; ...
%                  0.9978   -0.0054   -0.0655   89.2822; ...
%                  0.0574    0.5569    0.8286  275.6522; ...
%                       0         0         0    1.0000];

% convert camera xyPoints to checkerboard coords
checker_xy = pointsToWorld(camParams,T_cam_checker(1:3,1:3),T_cam_checker(1:3,4),camPoints);

checker_points = zeros(1,3);
disp("camPoints")
disp(camPoints)
disp("checker_xy")
disp(checker_xy)
checker_points(1,1:2) = checker_xy;

robot_coord = T_checker_robot(1:3,1:3) * checker_points.' + T_checker_robot(1:3,4);

end

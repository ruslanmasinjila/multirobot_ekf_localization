%   localizes a moving robot using EKF algorithm...
%   stationary robots act as landmarks to moving robots

function movingRobot = localizeRobot( movingRobot, stationaryRobot, ut_actual )

%   Localize robot using encoders only
movingRobot=moveRobot(movingRobot,ut_actual);

%   refine the estimated pose via relative measurements
movingRobot=correctPoseEstimates(movingRobot,stationaryRobot);
end


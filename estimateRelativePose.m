%   estimates the relative distance and orientation between...
%   moving robot, and stationary robot that acts as a landmark
function movingRobot = estimateRelativePose(movingRobot,stationaryRobot)

%######################### MEASURED DISTANCES AND ANGLES ##################

%   calculate measured relative distance

dx=stationaryRobot.groundTruth(end,1)-movingRobot.groundTruth(end,1);

dy=stationaryRobot.groundTruth(end,2)-movingRobot.groundTruth(end,2);

rng('shuffle');
rho=sqrt((dx)^2+(dy)^2)+movingRobot.sigma_rho*randn(1);

%   calculate measured relative angle

rng('shuffle');
phi=normalizeAngle(normalizeAngle(normalizeAngle(atan2(dy,dx))-...
        movingRobot.groundTruth(end,3))+...
            normalizeAngle(movingRobot.sigma_phi*randn(1)));
   
%   combine measured rho and phi

movingRobot.Z=[rho;phi];


%######################### ESTIMATED DISTANCES AND ANGLES ##################

%   calculate estimated relative distance

dx_bar=stationaryRobot.mu(end,1)-movingRobot.mu_bar(1);

dy_bar=stationaryRobot.mu(end,2)-movingRobot.mu_bar(2);

rho_bar=sqrt((dx_bar)^2+(dy_bar)^2);

%   calculate estimated relative angle

phi_bar=normalizeAngle(normalizeAngle(normalizeAngle(atan2(dy_bar,dx_bar))-...
            movingRobot.mu_bar(3)));
   
%   combine measured rho_bar and phi_bar

movingRobot.Z_bar=[rho_bar;phi_bar];

movingRobot.Z_diff=evaluateRelativePoseDifference(movingRobot.Z,movingRobot.Z_bar);


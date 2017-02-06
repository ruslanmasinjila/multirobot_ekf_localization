%   refines the pose estimated poses using relative measurements.

function movingRobot=correctPoseEstimates(movingRobot,stationaryRobot)

%##########################################################################

%   get relative pose estimates and measurements.
movingRobot = estimateRelativePose(movingRobot,stationaryRobot);

%##########################################################################

%   estimate the partial derivative of the observation/measurement w.r.t to
%   predicted state (priori)
[Hr, Hl]=evaluateMeasurementJacobians(movingRobot.mu_bar,stationaryRobot.mu(end,:));

%##########################################################################

%   Determine the covariance matrix of the sensors
Qt=getSensorCovariance([movingRobot.sigma_rho,movingRobot.sigma_phi]);

%##########################################################################

%   calculate innovation covariance
S=  ((Hr)*(movingRobot.sigma_bar)*(Hr')+...
        (Hl)*stationaryRobot.sigma{end}*(Hl')+...
        Qt);

%##########################################################################    

%   Calculate Kalman Gain (Kgain)
Kgain=  ((movingRobot.sigma_bar)*(Hr'))/S;   

%##########################################################################

%   Update mean for moving Robot
movingRobot.mu(end+1,:)=movingRobot.mu_bar+(Kgain*movingRobot.Z_diff)'; 

%##########################################################################

%   Normalize theta to [-pi,pi]
movingRobot.mu(end,3)=normalizeAngle(movingRobot.mu(end,3));

%##########################################################################

%   Update Covariance for moving Robot
movingRobot.sigma{end+1}=(eye(3,3)-Kgain*Hr)*movingRobot.sigma_bar;

%##########################################################################

end
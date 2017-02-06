%   1. calculates the moving robot's ground truth (zero encoder error) and
%       estimated position (priori)
%   2. propagates the uncertainty associated with robot's estimated (ekf) pose.

function movingRobot = moveRobot(movingRobot,ut_actual)


length_control=length(ut_actual(:,1));
b=movingRobot.b;

groundTruthX=movingRobot.groundTruth(end,1);
groundTruthY=movingRobot.groundTruth(end,2);
groundTruthTheta=movingRobot.groundTruth(end,3);

encoderPoseX=movingRobot.encoderPose(end,1);
encoderPoseY=movingRobot.encoderPose(end,2);
encoderPoseTheta=movingRobot.encoderPose(end,3);

mu_barX=movingRobot.mu(end,1);
mu_barY=movingRobot.mu(end,2);
mu_barTheta=movingRobot.mu(end,3);

sigma_bar=movingRobot.sigma{end};

%   get the distance traveled by the moving robot up to this point
distanceTraveled=movingRobot.distanceTraveled(end);

for i=1:length_control
    
%   get the actual control input for the left and right wheel.
DL_actual=ut_actual(i,1);
DR_actual=ut_actual(i,2);

%   increment the total distance travelled
distanceTraveled=distanceTraveled+(DL_actual+DR_actual)/2;
%##########################################################################

%   add some gaussian noise into the actual control input...
%   to simulate uncertainty in encoder data measured by the moving robot
DL_measured=ut_actual(i,1)+ut_actual(i,1)*movingRobot.KL*randn;
DR_measured=ut_actual(i,2)+ut_actual(i,2)*movingRobot.KR*randn;

%##########################################################################

%   compute the partial derivatives of the predicted state...
%   w.r.t measured control input (G_ut) and w.r.t previous filtered state (G_mut).
[ G_mut,G_ut] =...
    evaluatePredictionJacobians(movingRobot.b,mu_barTheta,[DL_measured,DR_measured]);

%   calculate odometry covariance as a function of encoder error and
%   the measured control input
Rt=getOdometryCovariance([DL_measured,DR_measured],[movingRobot.KL,movingRobot.KR]);

%   estimate the predicted (priori) covariance
sigma_bar=G_mut*sigma_bar*(G_mut)' + G_ut*Rt*(G_ut)';

%##########################################################################

%   calculate the actual new x coordinate
groundTruthX=   groundTruthX + ((DR_actual+DL_actual)/2)*cos(groundTruthTheta + ((DR_actual-DL_actual)/(2*b)));

%   calculate the measured new x coordinate
mu_barX=   mu_barX + ((DR_measured+DL_measured)/2)*cos(mu_barTheta + ((DR_measured-DL_measured)/(2*b)));

%   calculate the new encoder only x coordinate
encoderPoseX=   encoderPoseX + ((DR_measured+DL_measured)/2)*cos(encoderPoseTheta + ((DR_measured-DL_measured)/(2*b)));

%##########################################################################

%   calculate the actual new y coordinate
groundTruthY=  groundTruthY +  ((DR_actual+DL_actual)/2)*sin(groundTruthTheta+ ((DR_actual-DL_actual)/(2*b)));

%   calculate the measured new y coordinate
mu_barY=  mu_barY +  ((DR_measured+DL_measured)/2)*sin(mu_barTheta+ ((DR_measured-DL_measured)/(2*b)));

%   calculate the new encoder only y coordinate
encoderPoseY=  encoderPoseY +  ((DR_measured+DL_measured)/2)*sin(encoderPoseTheta+ ((DR_measured-DL_measured)/(2*b)));

%##########################################################################

%   calculate the actual new theta
groundTruthTheta=groundTruthTheta+(DR_actual-DL_actual)/b;

%   calculate the measured new theta
mu_barTheta=mu_barTheta+(DR_measured-DL_measured)/b;

%   calculate the new encoder only theta
encoderPoseTheta=encoderPoseTheta+(DR_measured-DL_measured)/b;

%##########################################################################


%   normalize angles between [-pi,pi]
groundTruthTheta=normalizeAngle(groundTruthTheta);
mu_barTheta=normalizeAngle(mu_barTheta);

end

movingRobot.groundTruth(end+1,:)=[groundTruthX,groundTruthY,groundTruthTheta];
movingRobot.encoderPose(end+1,:)=[encoderPoseX,encoderPoseY,encoderPoseTheta];
movingRobot.mu_bar=[mu_barX,mu_barY,mu_barTheta];
movingRobot.sigma_bar=sigma_bar;
movingRobot.distanceTraveled(end+1)=distanceTraveled;






end
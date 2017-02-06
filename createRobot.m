%   creates and initializes various robot parameters

function robot = createRobot( initialGroundTruth, initialPoseError, encoderError, sensorError, distanceBetweenWheels, robotID )

%   forces randn to produce different set of numbers...
%   each time the randn is called
rng('shuffle');

%   distance between the two wheels of the robot
robot.b=distanceBetweenWheels;

%   error in relative distance measurements
robot.sigma_rho=sensorError(1);

%   error in relative angle measurements
robot.sigma_phi=sensorError(2);

%   percentage error in wheel encoders
robot.KL=encoderError(1);
robot.KR=encoderError(2);

%   distance travelled by the robot
robot.distanceTraveled(1)=0;

%   initial ground truth of the robot
robot.groundTruth(1,:)=initialGroundTruth;

%   initial error in position
sigmaX=initialPoseError(1);
sigmaY=initialPoseError(2);

%   initial error in orientation
sigmaTheta=initialPoseError(3);

%   initial estimated position of the robot
robot.mu(1,:)=...
    initialGroundTruth+[sigmaX*randn(1),sigmaY*randn(1),sigmaTheta*randn(1)];

%   initial encoder only position of the robot
robot.encoderPose=robot.mu;

%   initial covariance matrix
robot.sigma{1}=[sigmaX^2,0,0;0,sigmaY^2,0;0,0,sigmaTheta^2];

%   INTERMEDIATE STATES

%   intermediate estimated position
robot.mu_bar=[];

%   intermediate estimated covariance matrix
robot.sigma_bar=[];


%   MEASUREMENT VARIABLES

%   estimated relative distance and orientation
robot.Z_bar=[];

%   measured relative distance and orientation
robot.Z=[];

%   difference between measured and estimated relative distance and
%   orientation
robot.Z_diff=[];


end


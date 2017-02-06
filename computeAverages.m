%   computes the average estimated trajectories, and estimated covariances
%   for numRuns runs

function robot = computeAverages( robotNRuns )

numRuns=length(robotNRuns);

%##########################################################################

%   initialize partial sums for actual, estimated and encoder poses to zero
groundTruthSum=zeros(size(robotNRuns(1).groundTruth));
muSum=zeros(size(robotNRuns(1).mu));
encoderPoseSum=zeros(size(robotNRuns(1).encoderPose));
aneesSum=zeros(1,length(robotNRuns(1).groundTruth(:,1)));
distanceTraveledSum=zeros(1,length(robotNRuns(1).distanceTraveled));

%   initialize partial sums for covariance matrices to zero
sigmaLength=length(robotNRuns(1).sigma);
for i=1:sigmaLength
    sigmaSum{i}=zeros(3,3);
end

%##########################################################################

%   add partial sums together
for i=1:numRuns

    groundTruthSum=groundTruthSum+robotNRuns(i).groundTruth;
    muSum=muSum+robotNRuns(i).mu;
    encoderPoseSum=encoderPoseSum+robotNRuns(i).encoderPose;
    distanceTraveledSum=distanceTraveledSum+robotNRuns(i).distanceTraveled;
    
    
    for j=1:sigmaLength
        
       sigmaSum{j}= sigmaSum{j}+robotNRuns(i).sigma{j};
       aneesSum(j)=aneesSum(j)+...
                    (robotNRuns(i).groundTruth(j,:)-robotNRuns(i).mu(j,:))*...
                        (inv(robotNRuns(i).sigma{j}))*...
                            (robotNRuns(i).groundTruth(j,:)-robotNRuns(i).mu(j,:))';                 
    end

end
%##########################################################################

%   compute the means of partial sums
robot.groundTruth=groundTruthSum/numRuns;
robot.mu=muSum/numRuns;
robot.encoderPose=encoderPoseSum/numRuns;
robot.anees=(1/3).*(aneesSum/numRuns);
robot.distanceTraveled=distanceTraveledSum/numRuns;

%   compute the actual error between estimated and actual poses...
%   averaged over the number of simulation runs.
robot.actualError=abs(robot.groundTruth-robot.mu);

for j=1:sigmaLength
    robot.sigma{j}= sigmaSum{j}/numRuns;
    
%   determine optimistic, pessimistic, and optimal ANEES values
[lower_bound,upper_bound] = anees_bounds(numRuns);
robot.optimistic_anees=find(robot.anees>upper_bound);
robot.pessimistic_anees=find(robot.anees<lower_bound);
robot.valid_anees=find(robot.anees>=lower_bound&robot.anees<=upper_bound);

%##########################################################################

end



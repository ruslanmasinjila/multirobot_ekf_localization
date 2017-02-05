%   computes the average estimated trajectories, and estimated covariances
%   for numRuns runs

function robot = computeAverages( robotNRuns )

numRuns=length(robotNRuns);


%   initialize partial sums to zero
groundTruthSum=zeros(size(robotNRuns(1).groundTruth));
muSum=zeros(size(robotNRuns(1).mu));
encoderPoseSum=zeros(size(robotNRuns(1).encoderPose));

sigmaLength=length(robotNRuns(1).sigma);
for i=1:sigmaLength
    sigmaSum{i}=zeros(3,3);
end



%   add partial sums together
for i=1:numRuns

    groundTruthSum=groundTruthSum+robotNRuns(i).groundTruth;
    muSum=muSum+robotNRuns(i).mu;
    encoderPoseSum=encoderPoseSum+robotNRuns(i).encoderPose;
    
    for j=1:sigmaLength
       sigmaSum{j}= sigmaSum{j}+robotNRuns(i).sigma{j};
    end

end

%   compute the means of partial sums
robot.groundTruth=groundTruthSum/numRuns;
robot.mu=muSum/numRuns;
robot.encoderPose=encoderPoseSum/numRuns;

%   compute the actual error between estimated and actual poses...
%   averaged over the number of simulation runs.
robot.actualError=abs(robot.groundTruth-robot.mu);

for j=1:sigmaLength
    robot.sigma{j}= sigmaSum{j}/numRuns;
end



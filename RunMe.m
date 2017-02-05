%   Change the number of robots

clear all;
close all;
clc;


%   Number of robots in a team
numRobots=5;

%   Discrete counter for the length of simulation
simulationLength=100;

%TODO [in version 3]
%   Number of times the simulation is repeated under the same initial...
%   and control conditions (for statistical analysis).
%   numRuns=1;

%##########################################################################

%   create robots with random initial ground truths...
%   and specified sensor errors.
for i=1:numRobots
%   robot = createRobot( initialgroundTruth, initialPoseError, encoderError, sensorError, distanceBetweenWheels)
    rng('shuffle');
    robots(i)= createRobot([5*randn,5*randn,normalizeAngle(5*rand)],[0.15,0.15,0.15],[0.02,0.02],[0.03,0.1],0.5);
end

%##########################################################################

%   randomly choose moving and stationary robots for each step
for i=1:simulationLength
    
    %   randomly choose up to (numRobots-1) robots to move
    movingRobots{i}=randsample(numRobots,randsample(numRobots-1,1));
    
    %   generate control inputs for each of the moving robots
    %   i is step
    %   j is moving robot
    for j=1:length(movingRobots{i})
        ut{i,j}=abs(0.1*randn(10,2));
    end
    
    %   the rest of the robots (at least one) remain stationary
    stationaryRobots{i}=setdiff(1:length(robots),movingRobots{i});
    
end

%##########################################################################

%   TODO [simulate numRuns times uner the same initial and control conditions]
%   collect data for ANEES calculations

%   simulate numRuns times
%for i=1:numRuns
    
    %   for each step within a run
    for j=1:simulationLength
        
        %   select a robot to move
        for k=1:length(movingRobots{j})
            
            movingRobot=movingRobots{j}(k);
            
            %   localize w.r.t first stationary robot.
            stationaryRobot=stationaryRobots{j}(1);
            robots(movingRobot)=localizeRobot(robots(movingRobot),robots(stationaryRobot), ut{j,k});
                        
            %   localize the selected robot w.r.t the rest of stationary robots
            for l=2:length(stationaryRobots{j})
                
                stationaryRobot=stationaryRobots{j}(l);
                robots(movingRobot)=localizeRobot(robots(movingRobot),robots(stationaryRobot),[0,0]);
                
            end
                        
        end
        
        
    end

%end

    

 %##########################################################################
 
 %  generate random colors for the graphs
 
 for i=1:length(robots)
     
     randomColor(i,:)=[rand,rand,rand];   
 end

% Ground Truths and EKF plots
figure;
set(gca,'fontsize',15);

for i=1:length(robots)
    
 
plot(robots(i).groundTruth(:,1),robots(i).groundTruth(:,2),'-o','color',randomColor(i,:));
hold on;
plot(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),'o','MarkerSize',12,'color',randomColor(i,:));
hold on;
text(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),['Robot ',num2str(i),' start'],'fontsize',12);
hold on;

plot(robots(i).mu(:,1),robots(i).mu(:,2),'--x','color',randomColor(i,:));
hold on;

end

title('Ground Truths (solid) and EKF Estimates (broken)');
xlabel('x (m)');
ylabel('y (m)');


%##########################################################################

% Ground Truths with Encoder only poses
figure;
set(gca,'fontsize',15);

for i=1:length(robots)
    
plot(robots(i).groundTruth(:,1),robots(i).groundTruth(:,2),'-o','color',randomColor(i,:));
hold on;
plot(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),'o','MarkerSize',12,'color',randomColor(i,:));
hold on;
text(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),['Robot ',num2str(i),' start'],'fontsize',12);
hold on;

plot(robots(i).encoderPose(:,1),robots(i).encoderPose(:,2),'--x','color',randomColor(i,:));
hold on;

end

title('Ground Truths (solid) and Encoder Estimates (broken)');
xlabel('x (m)');
ylabel('y (m)');

%##########################################################################




%   Change the number of robots

clear all;
close all;
clc;
%##########################################################################

%   Number of robots in a team
numRobots=5;

%   Discrete counter for the length of simulation (larger number, longer simulation)
simulationLength=50;


%   Number of times the simulation is repeated under the same initial...
%   and control conditions (for statistical analysis).
numRuns=1;

%##########################################################################

%   create robots with random initial ground truths...
%   and specified sensor errors.

for i=1:numRobots
    
%   robot = createRobot( initialgroundTruth, initialPoseError, encoderError, sensorError, distanceBetweenWheels)
    rng('shuffle');
    robots(i)= createRobot([5*randn,5*randn,normalizeAngle(5*randn)],[0.15,0.15,0.15],[0.02,0.02],[0.1,0.1],0.5);
    
end

%   create a copy of initialized robots for repeated simulations under...
%   the same initial starting conditions an sensor noise
robots_copy=robots;

%##########################################################################

%   randomly choose moving and stationary robots...
%   at each step of the simulation

for i=1:simulationLength
    
    %   randomly choose up to (numRobots-1) robots to move
    movingRobots{i}=randsample(numRobots,randsample(numRobots-1,1));
    
    %   generate control inputs for each of the moving robots
    for j=1:length(movingRobots{i})
        ut{i,j}=abs(0.1*randn(10,2));
    end
    
    %   the rest of the robots (at least one) remain stationary
    stationaryRobots{i}=setdiff(1:length(robots),movingRobots{i});
    
end

%##########################################################################

%   simulate numRuns times
for i=1:numRuns
    disp(['Run ',num2str(i), ' of ', num2str(numRuns)]);
    
    %   use a fresh copy of robots and subject them to the same motion...
    %   sequences and control inputs
    robots=robots_copy;
    
    %   for each step within a run
    for j=1:simulationLength
        
        %   select a robot to move
        
        for k=1:length(movingRobots{j})
            
            movingRobot=movingRobots{j}(k);
            
            %   localize the selected moving w.r.t first stationary robot.
            
            stationaryRobot=stationaryRobots{j}(1);
            
            robots(movingRobot)=localizeRobot(robots(movingRobot),robots(stationaryRobot), ut{j,k});
                        
            %   localize the selected moving robot w.r.t the rest of stationary robots
            
            for l=2:length(stationaryRobots{j})
                
                stationaryRobot=stationaryRobots{j}(l);
                
                robots(movingRobot)=localizeRobot(robots(movingRobot),robots(stationaryRobot),[0,0]);
                
            end
                        
        end
        
        
    end
    %   save the copies of robots for statistical analysis (evaluation of ANEES)
    robotsNRuns(i,:)=robots;
end

%##########################################################################

%   compute averages.
for i=1:length(robots)
    
    robotsAverage(i)=computeAverages(robotsNRuns(:,i));
    
end

%##########################################################################

%   draw various graphs
makePlots(robotsAverage,numRuns);

%##########################################################################


%   Display ANEES percentages
for i=1:length(robotsAverage)

pessimistic_anees=(length(robotsAverage(i).pessimistic_anees)/length(robotsAverage(i).anees))*100;
optimistic_anees=(length(robotsAverage(i).optimistic_anees)/length(robotsAverage(i).anees))*100;
valid_anees=(length(robotsAverage(i).valid_anees)/length(robotsAverage(i).anees))*100;

disp('*******************************');
disp(['ANEES FOR ROBOT ',num2str(i)]);
disp(['Pessimistic ANEES: ',num2str(pessimistic_anees),'%']);
disp(['Optimistic ANEES ',num2str(optimistic_anees),'%']);
disp(['Valid ANEES ',num2str(valid_anees),'%']);

end







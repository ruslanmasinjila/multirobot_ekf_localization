%   plots various types of graphs and displays information on terminal

function makePlots( robots, numRuns )

%   The ANEES can have outliers which could potentially distort the
%   appearance of its curves. Keep outlierThreshold 1000 for
%   simulationLength<=50. Adjust as necessary.
outlierThreshold=25;

%##########################################################################
 
 %  generate random colors for the graphs
 
 for i=1:length(robots)
     
     randomColor(i,:)=[rand,rand,0];   
 end
 
 %##########################################################################

% Ground Truths and EKF 
figure;
set(gca,'fontsize',15);

for i=1:length(robots)
    
 
plot(robots(i).groundTruth(:,1),robots(i).groundTruth(:,2),'-o','color',randomColor(i,:));
hold on;

plot(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),'o','MarkerSize',12,'color',randomColor(i,:));
hold on;

text(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),['R',num2str(i),' start'],'fontsize',12,'fontweight','bold');
hold on;

plot(robots(i).mu(:,1),robots(i).mu(:,2),'--x','color',randomColor(i,:));
hold on;

end

title(['Ground Truths (solid-o) and EKF Estimates (broken-x) of ',num2str(length(robots)),' Robots']);
xlabel('x (m)');
ylabel('y (m)');


%##########################################################################

% Ground Truths and encoder only
figure;
set(gca,'fontsize',15);

for i=1:length(robots)
    
plot(robots(i).groundTruth(:,1),robots(i).groundTruth(:,2),'-o','color',randomColor(i,:));
hold on;

plot(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),'o','MarkerSize',12,'color',randomColor(i,:));
hold on;

text(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),['R',num2str(i),' start'],'fontsize',12,'fontweight','bold');
hold on;

plot(robots(i).encoderPose(:,1),robots(i).encoderPose(:,2),'--x','color',randomColor(i,:));
hold on;

end

title(['Ground Truths (solid-o) and Encoder Estimates (broken-x) of ',num2str(length(robots)),' Robots']);
xlabel('x (m)');
ylabel('y (m)');

%##########################################################################

% Average Normalized Estimation Error Squared
figure;
set(gca,'fontsize',15);

%   draw the upper and lower anees bounds
[lower_bound,upper_bound] = anees_bounds(numRuns);


for i=1:length(robots)
    
%   remove outliers in anees
indices = robots(i).anees>outlierThreshold;
robots(i).anees(indices) = [];

%   make a copy of robots(i).distanceTraveled.
%   resize the copy to match the dimensions of the robots(i).anees vector

distanceTraveledANEES=robots(i).distanceTraveled;
distanceTraveledANEES(indices)=[];
    
plot(distanceTraveledANEES,upper_bound.*ones(1,length(distanceTraveledANEES)),'--k');
hold on;

plot(distanceTraveledANEES,robots(i).anees,'-x','color',randomColor(i,:));
hold on;

plot(distanceTraveledANEES,lower_bound.*ones(1,length(distanceTraveledANEES)),'--k');
hold on;

text(distanceTraveledANEES(end),robots(i).anees(end),['R',num2str(i)],'fontsize',12,'fontweight','bold');
hold on;


end

title(['Average Normalized Estimation Errors of ',num2str(length(robots)),' Robots']);
xlabel('Distance Moved by Robot (m)');
ylabel('ANEES');

%##########################################################################

% absolute errors in x coordinates
figure;
set(gca,'fontsize',15);

for i=1:length(robots)
    
 
plot(robots(i).distanceTraveled,robots(i).actualError(:,1),'-x','color',randomColor(i,:));
hold on;

text(robots(i).distanceTraveled(end),robots(i).actualError(end,1),['R',num2str(i)],'fontsize',12,'fontweight','bold');
hold on;

end

title(['Actual Errors in X-Axis of ',num2str(length(robots)),' Robots']);
xlabel('Distance Moved by Robot');
ylabel('Error (m)');

%##########################################################################

% absolute errors in y coordinates
figure;
set(gca,'fontsize',15);

for i=1:length(robots)
    
 
plot(robots(i).distanceTraveled,robots(i).actualError(:,2),'-x','color',randomColor(i,:));
hold on;

text(robots(i).distanceTraveled(end),robots(i).actualError(end,2),['R',num2str(i)],'fontsize',12,'fontweight','bold');
hold on;

end

title(['Actual Errors in Y-Axis of ',num2str(length(robots)),' Robots']);
xlabel('Distance Moved by Robot (m)');
ylabel('Error (m)');

%##########################################################################

end


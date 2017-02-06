function makePlots( robots, numRuns )

%   The ANEES can have outliers which could potentially distort the
%   appearance of its curves. Keep outlierThreshold 1000 for
%   simulationLength<=50. Adjust as necessary.
outlierThreshold=1000;

%##########################################################################
 
 %  generate random colors for the graphs
 
 for i=1:length(robots)
     
     randomColor(i,:)=[rand,rand,rand];   
 end
 
 %##########################################################################

% Ground Truths and EKF plots
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

text(robots(i).groundTruth(1,1),robots(i).groundTruth(1,2),['R',num2str(i),' start'],'fontsize',12,'fontweight','bold');
hold on;

plot(robots(i).encoderPose(:,1),robots(i).encoderPose(:,2),'--x','color',randomColor(i,:));
hold on;

end

title('Ground Truths (solid) and Encoder Estimates (broken)');
xlabel('x (m)');
ylabel('y (m)');

%##########################################################################

% Average Normalized Estimation Error Squared
figure;
set(gca,'fontsize',15);

%   draw the upper and lower anees bounds
[lower_bound,upper_bound] = anees_bounds(numRuns);

for i=1:length(robots)
    
%   remove outlier anees
indices = robots(i).anees>1000;
robots(i).anees(indices) = [];
    
plot(upper_bound.*ones(1,length(robots(i).anees)),'--k');
hold on;

plot(robots(i).anees,'-x','color',randomColor(i,:));
hold on;

plot(lower_bound.*ones(1,length(robots(i).anees)),'--k');
hold on;

text(length(robots(i).anees),robots(i).anees(end),['R',num2str(i)],'fontsize',12,'fontweight','bold');
hold on;


end

title('AVERAGE NORMALIZED ESTIMATION ERROR SQUARED');
xlabel('Iteration');
ylabel('ANEES');

%##########################################################################

end


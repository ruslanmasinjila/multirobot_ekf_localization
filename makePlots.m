function makePlots( robots )

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

end


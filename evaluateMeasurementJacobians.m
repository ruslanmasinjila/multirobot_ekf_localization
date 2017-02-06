%   Computes the Jacobians Hr and Hl
%   Hr:     Partial derivative of the Estimated Relative Pose w.r.t current pose
%   Hl:     Partial derivetive of the Estimated Relative Pose w.r.t
%           landmark pose


%   INPUT:  
%   Current Pose of the Robot (currentPose)
%   Landmark Pose (landmarjPose)

%   OUTPUT:
%   Hr
%   Hl

function [ Hr,Hl] = evaluateMeasurementJacobians( currentPose, landmarkPose)


    lx=landmarkPose(1);
    ly=landmarkPose(2);
    
    rx=currentPose(1);
    ry=currentPose(2);
    
    Hr(1,1)=-(2*lx - 2*rx)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2));
    Hr(1,2)=-(2*ly - 2*ry)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2));
    Hr(1,3)=0;
    Hr(2,1)=-(2*((2*lx - 2*rx)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2)) - 1))/(((rx - lx + ((lx - rx)^2 + (ly - ry)^2)^(1/2))^2/(ly - ry)^2 + 1)*(ly - ry));
    Hr(2,2)=(2*((rx - lx + ((lx - rx)^2 + (ly - ry)^2)^(1/2))/(ly - ry)^2 - (2*ly - 2*ry)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2)*(ly - ry))))/((rx - lx + ((lx - rx)^2 + (ly - ry)^2)^(1/2))^2/(ly - ry)^2 + 1);
    Hr(2,3)=-1;
    
    Hl(1,1)=(2*lx - 2*rx)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2));
    Hl(1,2)=(2*ly - 2*ry)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2));
    Hl(1,3)=0;
    Hl(2,1)=(2*((2*lx - 2*rx)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2)) - 1))/(((rx - lx + ((lx - rx)^2 + (ly - ry)^2)^(1/2))^2/(ly - ry)^2 + 1)*(ly - ry));
    Hl(2,2)=-(2*((rx - lx + ((lx - rx)^2 + (ly - ry)^2)^(1/2))/(ly - ry)^2 - (2*ly - 2*ry)/(2*((lx - rx)^2 + (ly - ry)^2)^(1/2)*(ly - ry))))/((rx - lx + ((lx - rx)^2 + (ly - ry)^2)^(1/2))^2/(ly - ry)^2 + 1);
    Hl(2,3)=0;
    
       

end

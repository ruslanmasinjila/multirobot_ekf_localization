%   Computes the Jacobians G_mut and G_ut
%   G_mut:  Partial derivative of the current pose w.r.t previous pose
%   G_ut:   Partial derivetive of the current pose w.r.t measured control input

%   INPUT:  
%   Previous Pose of the Robot (mu)
%   Control Input (ut)

%   OUTPUT:
%   G_mut
%   G_ut

function [ G_mut,G_ut] = evaluatePredictionJacobians(b,previousTheta,ut_measured)


DL=ut_measured(1);
DR=ut_measured(2);


DS=(DR+DL)/2;
DT=normalizeAngle((DR-DL)/(2*b));


G_mut(1,:)=[ 1, 0, -(DS)*sin(normalizeAngle(previousTheta+(DT)))];
G_mut(2,:)=[ 0, 1,  (DS)*cos(normalizeAngle(previousTheta+(DT)))];
G_mut(3,:)=[0 0 1];


G_ut(1,:)=[DS*sin(normalizeAngle(previousTheta+DT))+b*cos(normalizeAngle(previousTheta+DT)), -DS*sin(normalizeAngle(previousTheta+DT))+b*cos(normalizeAngle(previousTheta+DT))];
G_ut(2,:)=[-DS*cos(normalizeAngle(previousTheta+DT))+b*sin(normalizeAngle(previousTheta+DT)), DS*cos(normalizeAngle(previousTheta+DT))+b*sin(normalizeAngle(previousTheta+DT))];
G_ut(3,:)=[-2, 2];

G_ut=(1/(2*b))*G_ut;


end



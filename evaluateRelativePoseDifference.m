
%   Calculates the difference between Estimated (Z_bar) and measured (Z)
%   poses followed by normalization of the results

%   INPUT:
%   Measured noisy relative position of a landmark with respect to robot (Z)
%   Estimated relative postion of a landmark with respect to a robot (Z_bar)
%
%   OUTPUT
%   Normalized Difference between Z and Z_bar

function Z_diff =evaluateRelativePoseDifference(Z,Z_bar)



Z_diff=Z-Z_bar;

%   Normalize the difference to [-pi,pi]
Z_diff(2)=normalizeAngle(Z_diff(2));


end

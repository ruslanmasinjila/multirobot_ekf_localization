%   Keeps angles between [-pi,pi]

%   INPUT:
%   original angle

%   OUTPUT
%   Normalized angle

function [ normalizedAngle ] = normalizeAngle( originalAngle )


normalizedAngle=mod(originalAngle,2*pi);

if(normalizedAngle>pi&&normalizedAngle<=2*pi)
    normalizedAngle=-(2*pi-normalizedAngle);
end




end


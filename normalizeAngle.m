%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function [ normalizedAngle ] = normalizeAngle( originalAngle )

%   Keeps angles between [-pi,pi]

%   INPUT:
%   original angle

%   OUTPUT
%   Normalized angle

%   BEGIN
normalizedAngle=mod(originalAngle,2*pi);

if(normalizedAngle>pi&&normalizedAngle<=2*pi)
    normalizedAngle=-(2*pi-normalizedAngle);
end
%   END



end


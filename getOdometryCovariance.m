function Rt = getOdometryCovariance(ut,encoderError)

%   Returns the covariance matrix of the wheel odometers

%   INPUT:
%   measured control input (ut=[DL DR])
%                DL~Left wheel displacement
%                DR~Right wheel displacement

%   Odometry Error Constants (K=[KL KR])
%                KL~Left wheel error
%                KR~Right wheel error
%
%   OUTPUT:
%   Odometer Covariance Matrix: Rt

KL=encoderError(1);
KR=encoderError(2);

DL=ut(1);
DR=ut(2);

Rt=[(KL*DL)^2 0;0 (KR*DR)^2];

%   END


end


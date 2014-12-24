function xhat = kalman(z, A, C, R, Q)
% Simple Kalman Filter (linear) with optimal gain, no control signal
%
% z Measurement signal              m observations X # of observations
% A State transition model          n X n, n = # of state values
% C Observation model               m X n
% R Covariance of observation noise m X m
% Q Covariance of process noise     n X n
%
% Based on http://en.wikipedia.org/wiki/Kalman_filter, but matrices named
% A, C, G instead of F, H, K.
%
% See http://home.wlu.edu/~levys/kalman_tutorial for background
%
% Copyright (C) 2014 Simon D. Levy
%
% This code is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as
% published by the Free Software Foundation, either version 3 of the
% License, or (at your option) any later version.
%
% This code is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this code. If not, see <http:#www.gnu.org/licenses/>.

% Initializtion =====================================================

% Number of sensors
m = size(C, 1);

% Number of state values
n = size(C, 2);

% Number of observations
numobs = size(z, 2);

% Use linear least squares to estimate initial state from initial
% observation
xhat = zeros(n, numobs);
xhat(:,1) = C \ z(:,1);

% Initialize P, I
P = ones(size(A));
I = eye(size(A));

% Kalman Filter =====================================================

for k = 2:numobs
    
    % Predict
    xhat(:,k) = A * xhat(:,k-1);
    P         = A * P * A' + Q;
    
    % Update
    G         = P  * C' / (C * P * C' + R);
    P         = (I - G * C) * P;
    xhat(:,k) = xhat(:,k) + G * (z(:,k) - C * xhat(:,k));
    
end

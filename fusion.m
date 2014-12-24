function fusion
% Kalman Filter sensor fusion example based on
%  
% http://www.slideshare.net/antoniomorancardenas/data-fusion-with-kalman-filtering
%
% % See http://home.wlu.edu/~levys/kalman_tutorial for background
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

% Parameters ============================================================

% Biases
BIAS1 = +1;
BIAS2 = -1;

% Measurement noise covariances
R1 = 0.64;
R2 = 0.64;

% Process noise covariance
Q = .005;

% State transition model
A = 1;

% Observation model
C1 = 1;
C2 = 1;

% Duration
N = 1000;

% Run ====================================================================

% Generate the signal x as a sine wave
x = 20 + sin(5*linspace(0,1,N)*pi);

% Add some process noise with covariance Q
w = sqrt(Q) * randn(size(x));
x = x + w;

% Compute noisy sensor values
z1 = BIAS1 + x + sqrt(R1) * randn(size(x));
z2 = BIAS2 + x + sqrt(R2) * randn(size(x));

% Run a single-sensor example and plot it
xhat = kalman(z1, A, C1, R1, Q);

% Plot sensed and estimated values
clf
plotsigs(1, z1, xhat, 'Sensor 1')
title('One sensor: signal cleanup')

% Plot estimate and actual values
plotsigsrms('One sensor', 2, x, xhat)

% Run the Kalman filter on fused sensors
xhat = kalman([z1; z2], A, [C1; C2], [R1 0; 0 R2], Q);

% Plot fusion example
plotsigsrms('Two sensors', 3, x, xhat)


% Helper functions ======================================================

function plotsigs(pos, sig1, sig2, sig1label)
subplot(3,1,pos)
hold on
plot(sig1, 'k')
plot(sig2, 'r')
legend({sig1label, 'Estimated'})
ylim([15 25])
hold off

function plotsigsrms(label, pos, x, xhat)
plotsigs(pos, x, xhat, 'Actual')
title(sprintf('%s: RMS error = %f', label, sqrt(sum((x-xhat).^2)/length(x))))



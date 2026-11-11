function [vs, ws, stop] = controlParkingLim(q, goalPose, goalId, vehicleId, collision, inside, map)
% Control the robot with steering limited to 30 degrees.
% q = [x; y; phi; gamma]. Commands are in m/s and rad/s.
persistent state
D = 0.1207;
kGamma = 14;
gammaMax = pi/6;
[target, speed, stop, state] = parkingStep(q, goalPose, goalId, vehicleId, ...
    collision, inside, map, gammaMax, state);

vs = 0;
ws = 0;
if isempty(target)
    return;
end

% Pure pursuit gives the steering reference from the look-ahead point.
q = state.q;
delta = target-q(1:2);
lateral = [-sin(q(3)), cos(q(3))]*delta;
gammaRef = atan(D*2*lateral/max(delta.'*delta, 1e-8));
gammaRef = min(max(gammaRef, -gammaMax), gammaMax);
eGamma = wrapToPi(gammaRef-q(4));
if strcmp(state.phase, 'steer') && abs(eGamma) > 0.018
    ws = min(max(kGamma*eGamma, -5), 5);
    return;
end
state.phase = 'drive';

% Convert rear-axle speed and steering rate to front-unit commands.
vs = min(max(speed/cos(gammaRef), -0.19), 0.19);
ws = min(max(vs*sin(q(4))/D + kGamma*eGamma, -5), 5);
end

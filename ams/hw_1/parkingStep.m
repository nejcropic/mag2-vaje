function [target, speed, stop, state] = parkingStep(q, goal, goalId, vehicleId, ...
    collision, inside, map, gammaMax, state)
% Update the shared maneuver and return its look-ahead point and signed speed.
headingTolerance = 0.45*pi/180;
target = [];
speed = 0;
stop = false;

if isempty(state) || state.vehicleId ~= vehicleId || state.goalId ~= goalId
    state = struct('vehicleId', vehicleId, 'goalId', goalId, 'q', q, ...
        'path', [], 'phase', 'settle', 'samples', 0, 's', 0, 'corrections', 0);
end
e = q-state.q;
e(3:4) = wrapToPi(e(3:4));
gain = 0.4;
if ~isempty(state.path) && abs(state.path(5,1)-state.s) < 0.03 && ...
        (size(state.path,2) == 1 || abs(state.path(5,1)) <= 0.033)
    gain = 0.15;
end
state.q = state.q + gain*e;
q = state.q;

if collision && strcmp(state.phase, 'drive')
    state.path = [];
    state.phase = 'settle';
    state.samples = 0;
end

if strcmp(state.phase, 'settle')
    state.samples = state.samples + 1;
    if isempty(state.path)
        if state.samples == 25
            state.mean = q;
        elseif state.samples > 25
            e = q-state.mean;
            e(3:4) = wrapToPi(e(3:4));
            state.mean = state.mean + e/(state.samples-24);
        end
        samplesNeeded = 65 + 35*(state.corrections > 0);
        if state.samples < samplesNeeded
            return;
        end
        q = state.mean;
        state.q = q;
        e = goal-q(1:3);
        e(3) = wrapToPi(e(3));
        if inside && norm(e(1:2)) <= 0.00082 && abs(e(3)) <= headingTolerance
            stop = true;
            return;
        end
        forward = [cos(q(3)); sin(q(3))];
        along = forward.'*e(1:2);
        across = [-forward(2), forward(1)]*e(1:2);
        if inside && abs(across) < 0.0006 && abs(e(3)) < headingTolerance ...
                && abs(along) > 0.00045 && abs(along) < 0.02
            state.path = [q(1:3); 0; along];
        end
        state.corrections = state.corrections + 1;
        if state.corrections > 8
            warning('Parking corrections did not converge for P%d.', goalId);
            state.phase = 'failed';
            return;
        end
    elseif state.samples < 22
        return;
    end
    state.phase = 'plan';
end

if strcmp(state.phase, 'plan')
    if isempty(state.path)
        state.path = parkingManeuver(q(1:3), goal, goalId, map, gammaMax);
        if isempty(state.path)
            warning('No clear maneuver found for P%d.', goalId);
            state.phase = 'failed';
            return;
        end
    end
    state.phase = 'steer';
    state.s = 0;
end
if strcmp(state.phase, 'failed')
    return;
end

segment = state.path(:,1);
phi0 = segment(3);
k = segment(4);
direction = sign(segment(5));
left0 = [-sin(phi0); cos(phi0)];
if k == 0
    state.s = [cos(phi0), sin(phi0)]*(q(1:2)-segment(1:2));
else
    center = segment(1:2) + left0/k;
    left = k*(center-q(1:2));
    phi = atan2(-left(1), left(2));
    state.s = state.s + wrapToPi(phi-phi0-k*state.s)/k;
end
remaining = direction*(segment(5)-state.s);
last = size(state.path,2) == 1;
shortMove = abs(segment(5)) <= 0.033;
tolerance = 0.0010;
if last || shortMove
    tolerance = 0.00025;
end
if remaining <= tolerance
    state.path(:,1) = [];
    state.phase = 'settle';
    state.samples = 0;
    return;
end

% Pure pursuit with a signed look-ahead works in both directions.
s = state.s + direction*0.046;
if k == 0
    target = segment(1:2) + s*[cos(phi0); sin(phi0)];
else
    phi = phi0+k*s;
    target = segment(1:2) + (left0-[-sin(phi); cos(phi)])/k;
end
speed = min([0.17, 3.6*remaining, 0.085+0.085/(1+0.5*abs(k))]);
if (last || shortMove) && remaining < 0.02
    speed = max(0.002, speed);
end
speed = direction*speed;
end

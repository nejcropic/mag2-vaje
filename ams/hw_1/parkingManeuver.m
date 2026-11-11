function path = parkingManeuver(start, goal, goalId, map, gammaMax)
% Fixed approach poses and parking maneuvers. Each segment is [x;y;phi;k;s].
D = 0.1207;
rMin = D/tan(gammaMax-pi/90);
radii = unique(max(rMin, [0.08, 0.13, 0.23, 0.32, 0.46]));
obstacles = [map.obstacles, map.vehicleObstacles];
rect = zeros(6, numel(obstacles));
for i = 1:numel(obstacles)
    p = obstacles{i};
    edge = p(:,2)-p(:,1);
    rect(:,i) = [mean(p,2); edge/norm(edge); norm(edge)/2; norm(p(:,4)-p(:,1))/2];
end

% Intermediate poses are set by the parking-lot layout.
if goalId <= 3
    offsets = [-0.16, 0.16, -0.28, 0.28];
    aisle = [0.40, 0.53; 0.86, 0.63; 0, pi/2];
elseif goalId == 11
    offsets = [-0.14, -0.24, -0.07];
    aisle = [0.40, 0.98; 0.86, 0.86; 0, pi];
elseif goalId >= 7
    offsets = [0.16, -0.16, 0.28, -0.28];
    aisle = [0.98, 0.85; 0.86, 0.63; 0, pi/2];
else
    offsets = [0.025, -0.025, 0.01, 0];
    aisle = [0.98, 0.69, 0.40; 0.86, 0.88, 0.86; 0, 0, 0];
end

path = [];
forward = [cos(goal(3)); sin(goal(3))];
e = start(1:2)-goal(1:2);
ePhi = wrapToPi(start(3)-goal(3));
if norm(e) < 0.025 && abs(ePhi) < pi/36
    along = forward.'*e;
    across = [-forward(2), forward(1)]*e;
    margin = 0.003+abs(across)+(map.H-map.I)*abs(sin(ePhi));
    from = [goal(1:2)+along*forward; goal(3)];
    for offset = offsets(abs(offsets) >= 0.14)
        entry = [goal(1:2)+offset*forward; goal(3)];
        retry = [[from; 0; offset-along], [entry; 0; -offset]];
        if clearPath(retry, rect, map, margin)
            path = retry;
            return;
        end
    end
end

% Join the measured pose to the selected approach with a local arc-line-arc maneuver.
for margin = [0.006, 0.002]
    best = inf;
    for offset = offsets
        entry = [goal(1:2)+offset*forward; goal(3)];
        last = [entry; 0; -offset];
        if ~clearPath(last, rect, map, margin)
            continue;
        end
        first = approach(start, entry, aisle, radii, rect, map, margin);
        if isempty(first)
            continue;
        end
        score = cost(first)+abs(offset)+0.10*(abs(offset)<0.02);
        if score < best
            best = score;
            path = [first, last];
        end
    end
    if ~isempty(path)
        path = path(:,abs(path(5,:)) >= 1e-7);
        return;
    end
end

if goalId >= 4 && goalId <= 6
    for moves = bottomMoves(goalId, gammaMax)
        last = fromGoal(goal, moves{1}, radii(1));
        for margin = [0.004, 0.0015, 0.0003]
            if ~clearPath(last, rect, map, margin)
                continue;
            end
            first = approach(start, last(1:3,1), aisle, radii, rect, map, margin);
            if ~isempty(first)
                path = [first, last];
                path = path(:,abs(path(5,:)) >= 1e-7);
                return;
            end
        end
    end
end
end

function path = approach(start, entry, aisle, radii, rect, map, margin)
path = connectPoses(start, entry, radii, rect, map, margin);
if ~isempty(path)
    return;
end
for mid = aisle
    last = connectPoses(mid, entry, radii, rect, map, margin);
    if isempty(last)
        continue;
    end
    first = connectPoses(start, mid, radii, rect, map, margin);
    if ~isempty(first)
        path = [first, last];
        return;
    end
end
end

function sets = bottomMoves(goalId, gammaMax)
% For a tight space, reverse into the bay and straighten with short shunts.
% Columns specify turn direction and signed travel in millimeters.
if gammaMax > pi/6
    straighten = [-1, 8; 1, -16; -1, 8];
    switch goalId
        case 4
            moves = [1, -65; -1, -65; 1, -130; straighten];
        case 5
            moves = [0, -65; 1, -65; -1, -130; -straighten(:,1), straighten(:,2)];
        case 6
            moves = [-1, -16; 0, -73; -1, -65; 1, -130; straighten];
    end
    sets = {moves};
else
    straighten = [-1, 8; 1, -8; -1, 8; 1, -8; -1, 8; ...
                   1, -16; -1, 16; 1, -16; -1, 8];
    switch goalId
        case 4
            sets = {[-1, -163; 1, -163; straighten]};
        case 5
            finish = [-straighten(:,1), straighten(:,2)];
            sets = {[-1, -228; finish], [-1, -163; finish]};
        case 6
            sets = {[1, -130; -1, 8; 1, -98; -1, 8; 1, -65; -1, 49], ...
                    [-1, -65; 0, -65; 1, -195; -1, 8; 1, -57; -1, 8; 1, -16; straighten]};
    end
end
end

function path = fromGoal(goal, moves, radius)
% Integrate the fixed moves backwards to anchor the entry to the goal pose.
path = zeros(5, size(moves,1));
q = goal;
for i = size(moves,1):-1:1
    k = moves(i,1)/radius;
    s = moves(i,2)/1000;
    q = segmentPose(q, k, -s);
    path(:,i) = [q; k; s];
end
end

function path = connectPoses(a, b, radii, rect, map, margin)
% Tangents between turning circles give arc, straight, arc maneuvers.
path = [];
best = inf;
leftA = [-sin(a(3)); cos(a(3))];
leftB = [-sin(b(3)); cos(b(3))];
delta = b(1:2) - a(1:2);
if abs(leftA.'*delta) < 1e-8 && abs(wrapToPi(b(3)-a(3))) < 1e-8
    straight = [a; 0; [cos(a(3)), sin(a(3))]*delta];
    if clearPath(straight, rect, map, margin)
        path = straight;
        return;
    end
end
for radius = radii
    for rA = [-radius, radius]
        cA = a(1:2) + rA*leftA;
        for rB = [-radius, radius]
            cB = b(1:2) + rB*leftB;
            d = cB-cA;
            distance = norm(d);
            if distance < 1e-9 && rA == rB
                candidate = [a; 1/rA; rA*wrapToPi(b(3)-a(3))];
                if cost(candidate) < best && clearPath(candidate, rect, map, margin)
                    path = candidate;
                    best = cost(candidate);
                end
                continue;
            end
            if distance < 1e-9 || abs(rB-rA) > distance
                continue;
            end
            % The straight section must be perpendicular to both radii.
            angle = atan2(d(2), d(1));
            opening = acos((rB-rA)/distance);
            for phi = angle + [-opening, opening] - pi/2
                left = [-sin(phi); cos(phi)];
                pA = cA-rA*left;
                pB = cB-rB*left;
                s = [cos(phi), sin(phi)]*(pB-pA);
                candidate = [[a; 1/rA; rA*wrapToPi(phi-a(3))], ...
                             [pA; phi; 0; s], ...
                             [pB; phi; 1/rB; rB*wrapToPi(b(3)-phi)]];
                if cost(candidate) < best && clearPath(candidate, rect, map, margin)
                    path = candidate;
                    best = cost(candidate);
                end
            end
        end
    end
end
end

function value = cost(path)
lengths = abs(path(5,:));
value = sum(lengths) + 0.045*nnz(lengths > 1e-7);
end

function ok = clearPath(path, rect, map, margin)
ok = false;
for segment = path
    s = linspace(0, segment(5), max(2, ceil(abs(segment(5))/0.006)+1));
    q = segmentPose(segment(1:3), segment(4), s);
    phi = q(3,:);
    forward = [cos(phi); sin(phi)];
    left = [-sin(phi); cos(phi)];
    center = q(1:2,:) + (map.H/2-map.I)*forward;
    h = map.H/2 + margin;
    w = map.W/2 + margin;
    reach = h*abs(forward) + w*abs(left);
    if any(center-reach < 0 | center+reach > [map.width; map.height], 'all')
        return;
    end
    % The four rectangle axes are enough to check for separation.
    for obstacle = rect
        d = center-obstacle(1:2);
        u = obstacle(3:4);
        v = [-u(2); u(1)];
        a = abs(u.'*forward);
        b = abs(u.'*left);
        du = abs(u.'*d);
        dv = abs(v.'*d);
        df = abs(sum(d.*forward,1));
        dl = abs(sum(d.*left,1));
        if any(du < obstacle(5)+h*a+w*b & dv < obstacle(6)+h*b+w*a & ...
                df < h+obstacle(5)*a+obstacle(6)*b & ...
                dl < w+obstacle(5)*b+obstacle(6)*a)
            return;
        end
    end
end
ok = true;
end

function q = segmentPose(start, k, s)
phi = start(3) + k*s;
if k == 0
    p = start(1:2) + [cos(start(3)); sin(start(3))].*s;
else
    left0 = [-sin(start(3)); cos(start(3))];
    p = start(1:2) + (left0-[-sin(phi); cos(phi)])/k;
end
q = [p; phi];
end

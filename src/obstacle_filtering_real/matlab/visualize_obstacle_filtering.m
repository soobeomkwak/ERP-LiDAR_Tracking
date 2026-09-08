function visualize_obstacle_filtering(waypointCsv)
% Real-time viewer for obstacle_filtering_real
% - Waypoint boundaries
% - Ego pose (map->base_link from /tf)
% - Obstacles: in=red, out=blue (representative points)
% - Mouse hover: obstacle details from /obstacle_in_debug,/obstacle_out_debug

if nargin < 1 || isempty(waypointCsv)
    error('Pass a waypoint CSV path, e.g. visualize_obstacle_filtering("path/to/fix_competition_format.csv")');
end

cfg.waypointCsv = waypointCsv;
cfg.mapFrame = 'map';
cfg.baseFrame = 'base_link';
cfg.inTopic = '/obstacle_in_repr';
cfg.outTopic = '/obstacle_out_repr';
cfg.inDebugTopic = '/obstacle_in_debug';
cfg.outDebugTopic = '/obstacle_out_debug';

[leftX, leftY, rightX, rightY] = loadWaypointsAsLocalENU(cfg.waypointCsv);

node = ros2node('/matlab_obstacle_debug');
subIn   = ros2subscriber(node, cfg.inTopic,     'sensor_msgs/PointCloud2', "Reliability","besteffort");
subOut  = ros2subscriber(node, cfg.outTopic,    'sensor_msgs/PointCloud2', "Reliability","besteffort");
subInDbg  = ros2subscriber(node, cfg.inDebugTopic,  'std_msgs/String', "Reliability","besteffort");
subOutDbg = ros2subscriber(node, cfg.outDebugTopic, 'std_msgs/String', "Reliability","besteffort");
subTf   = ros2subscriber(node, '/tf', 'tf2_msgs/TFMessage', "Reliability","besteffort");

fig = figure('Name', 'obstacle_filtering_real live', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
plot(ax, leftX, leftY, 'k-', 'LineWidth', 1.0);
plot(ax, rightX, rightY, 'k-', 'LineWidth', 1.0);
hIn = scatter(ax, nan, nan, 45, 'filled', 'MarkerFaceColor', [0.90 0.15 0.15]);
hOut = scatter(ax, nan, nan, 35, 'filled', 'MarkerFaceColor', [0.15 0.35 0.90]);
hEgo = plot(ax, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
legend(ax, {'Left WP', 'Right WP', 'IN', 'OUT', 'Ego'}, 'Location', 'best');
title(ax, 'map(ENU): IN(red), OUT(blue)');
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');

% Datacursor callback for hover details
set(hIn, 'UserData', struct('ids', [], 'info', containers.Map('KeyType', 'char', 'ValueType', 'any'), 'label', 'IN'));
set(hOut, 'UserData', struct('ids', [], 'info', containers.Map('KeyType', 'char', 'ValueType', 'any'), 'label', 'OUT'));
dcm = datacursormode(fig);
set(dcm, 'Enable', 'on', 'UpdateFcn', @datatipUpdateFn);

egoX = NaN;
egoY = NaN;
inInfo = containers.Map('KeyType', 'char', 'ValueType', 'any');
outInfo = containers.Map('KeyType', 'char', 'ValueType', 'any');

while isvalid(ax)
    tfMsg = subTf.LatestMessage;
    if ~isempty(tfMsg)
        [egoX, egoY] = parseEgoFromTf(tfMsg, cfg.mapFrame, cfg.baseFrame, egoX, egoY);
    end

    inDbgMsg = subInDbg.LatestMessage;
    if ~isempty(inDbgMsg)
        inInfo = parseDebugString(char(inDbgMsg.data));
    end
    outDbgMsg = subOutDbg.LatestMessage;
    if ~isempty(outDbgMsg)
        outInfo = parseDebugString(char(outDbgMsg.data));
    end

    inMsg = subIn.LatestMessage;
    outMsg = subOut.LatestMessage;

    [inX, inY, inIds] = parsePointCloudXYID(inMsg);
    [outX, outY, outIds] = parsePointCloudXYID(outMsg);

    set(hIn, 'XData', inX, 'YData', inY, ...
        'UserData', struct('ids', inIds, 'info', inInfo, 'label', 'IN'));
    set(hOut, 'XData', outX, 'YData', outY, ...
        'UserData', struct('ids', outIds, 'info', outInfo, 'label', 'OUT'));
    set(hEgo, 'XData', egoX, 'YData', egoY);

    drawnow limitrate;
    pause(0.05);
end

end

function [leftX, leftY, rightX, rightY] = loadWaypointsAsLocalENU(csvPath)
wp = readtable(csvPath);

if all(ismember({'L1_UTM_X', 'L1_UTM_Y', 'R1_UTM_X', 'R1_UTM_Y'}, wp.Properties.VariableNames))
    leftX = double(wp.L1_UTM_X);
    leftY = double(wp.L1_UTM_Y);
    rightX = double(wp.R1_UTM_X);
    rightY = double(wp.R1_UTM_Y);
elseif all(ismember({'L1_LAT', 'L1_LON', 'R1_LAT', 'R1_LON'}, wp.Properties.VariableNames))
    leftLat = double(wp.L1_LAT); leftLon = double(wp.L1_LON);
    rightLat = double(wp.R1_LAT); rightLon = double(wp.R1_LON);
    cLat0 = 0.5 * (leftLat(1) + rightLat(1));
    cLon0 = 0.5 * (leftLon(1) + rightLon(1));
    [leftX, leftY] = latlonToLocalENU(leftLat, leftLon, cLat0, cLon0);
    [rightX, rightY] = latlonToLocalENU(rightLat, rightLon, cLat0, cLon0);
else
    error('CSV must contain waypoint columns (UTM or LAT/LON pairs).');
end

cx0 = 0.5 * (leftX(1) + rightX(1));
cy0 = 0.5 * (leftY(1) + rightY(1));
leftX = leftX - cx0; leftY = leftY - cy0;
rightX = rightX - cx0; rightY = rightY - cy0;
end

function [egoX, egoY] = parseEgoFromTf(tfMsg, mapFrame, baseFrame, egoX, egoY)
for i = 1:numel(tfMsg.transforms)
    t = tfMsg.transforms(i);
    if strcmp(string(t.header.frame_id), mapFrame) && strcmp(string(t.child_frame_id), baseFrame)
        egoX = double(t.transform.translation.x);
        egoY = double(t.transform.translation.y);
        return;
    end
end
end

function [xs, ys, ids] = parsePointCloudXYID(msg)
xs = []; ys = []; ids = [];
if isempty(msg)
    return;
end

data = uint8(msg.data);
step = double(msg.point_step);
count = double(msg.width) * double(msg.height);
if step <= 0 || count <= 0
    return;
end

xOff = -1; yOff = -1; idOff = -1;
for i = 1:numel(msg.fields)
    f = msg.fields(i);
    name = char(f.name);
    if strcmp(name, 'x')
        xOff = double(f.offset);
    elseif strcmp(name, 'y')
        yOff = double(f.offset);
    elseif strcmp(name, 'id') || strcmp(name, 'cluster_id')
        idOff = double(f.offset);
    end
end
if xOff < 0 || yOff < 0
    return;
end

xs = zeros(count, 1);
ys = zeros(count, 1);
ids = -ones(count, 1);
for i = 1:count
    base = (i - 1) * step;
    bx = base + xOff + 1;
    by = base + yOff + 1;
    xs(i) = double(typecast(data(bx:bx+3), 'single'));
    ys(i) = double(typecast(data(by:by+3), 'single'));
    if idOff >= 0
        bid = base + idOff + 1;
        ids(i) = double(typecast(data(bid:bid+3), 'int32'));
    end
end
end

function infoMap = parseDebugString(s)
infoMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
if isempty(s)
    return;
end
lines = splitlines(string(s));
cur = struct();
hasCur = false;
for i = 1:numel(lines)
    line = strtrim(lines(i));
    if startsWith(line, '- obstacle[')
        if hasCur && isfield(cur, 'id')
            infoMap(num2str(cur.id)) = cur;
        end
        cur = struct();
        hasCur = true;
        continue;
    end

    if startsWith(line, 'id:')
        cur.id = str2double(extractAfter(line, 'id:'));
    elseif startsWith(line, 'source_cluster_id:')
        cur.source_cluster_id = str2double(extractAfter(line, 'source_cluster_id:'));
    elseif startsWith(line, 'distance:')
        cur.distance = str2double(extractAfter(line, 'distance:'));
    elseif startsWith(line, 'width:')
        cur.width = str2double(extractAfter(line, 'width:'));
    elseif startsWith(line, 'height:')
        cur.height = str2double(extractAfter(line, 'height:'));
    elseif startsWith(line, 'speed:')
        cur.speed = str2double(extractAfter(line, 'speed:'));
    elseif startsWith(line, 'heading_base_rad:')
        cur.heading_base = str2double(extractAfter(line, 'heading_base_rad:'));
    elseif startsWith(line, 'heading_map_rad:')
        cur.heading_map = str2double(extractAfter(line, 'heading_map_rad:'));
    end
end
if hasCur && isfield(cur, 'id')
    infoMap(num2str(cur.id)) = cur;
end
end

function out = datatipUpdateFn(~, eventObj)
pos = eventObj.Position;
t = get(eventObj.Target, 'UserData');
out = {sprintf('%s obstacle', t.label), sprintf('x: %.3f', pos(1)), sprintf('y: %.3f', pos(2))};

if isempty(t) || ~isfield(t, 'ids') || isempty(t.ids)
    return;
end

idx = eventObj.DataIndex;
if idx < 1 || idx > numel(t.ids)
    return;
end
id = t.ids(idx);
out{end+1} = sprintf('id: %d', id);

if ~isfield(t, 'info') || ~isa(t.info, 'containers.Map')
    return;
end
k = num2str(id);
if ~isKey(t.info, k)
    return;
end
info = t.info(k);

if isfield(info, 'source_cluster_id')
    out{end+1} = sprintf('source_cluster_id: %d', info.source_cluster_id);
end
if isfield(info, 'distance')
    out{end+1} = sprintf('distance: %.3f', info.distance);
end
if isfield(info, 'width')
    out{end+1} = sprintf('width: %.3f', info.width);
end
if isfield(info, 'height')
    out{end+1} = sprintf('height: %.3f', info.height);
end
if isfield(info, 'speed')
    out{end+1} = sprintf('speed: %.3f', info.speed);
end
if isfield(info, 'heading_base')
    out{end+1} = sprintf('heading_base: %.3f', info.heading_base);
end
if isfield(info, 'heading_map')
    out{end+1} = sprintf('heading_map: %.3f', info.heading_map);
end
end

function [x, y] = latlonToLocalENU(latDeg, lonDeg, lat0Deg, lon0Deg)
R = 6378137.0;
lat = deg2rad(double(latDeg));
lon = deg2rad(double(lonDeg));
lat0 = deg2rad(double(lat0Deg));
lon0 = deg2rad(double(lon0Deg));
x = (lon - lon0) .* cos(lat0) * R;
y = (lat - lat0) * R;
end

function plot_tracked_state_csv(csv_files, playback_rate)
% plot_tracked_state_csv Visualize tracked-state CSVs from analyze_tracked_state_bag.py.
%
% Usage:
%   plot_tracked_state_csv("/path/track_0_state.csv")
%   plot_tracked_state_csv(["/path/track_0_state.csv", "/path/track_689_state.csv"])
%   plot_tracked_state_csv(["/path/track_0_state.csv", "/path/track_689_state.csv"], 1.0)
%   plot_tracked_state_csv(["/path/track_0_state.csv", "/path/track_689_state.csv"], 0)

if nargin < 1 || isempty(csv_files)
    error('csv_files is required');
end
if nargin < 2
    playback_rate = 1.0;
end

if ischar(csv_files) || isstring(csv_files)
    csv_files = string(csv_files);
end
csv_files = reshape(string(csv_files), 1, []);

tracks = {};
for i = 1:numel(csv_files)
    csv_path = csv_files(i);
    if ~isfile(csv_path)
        error('CSV not found: %s', csv_path);
    end

    tbl = readtable(csv_path);
    if isempty(tbl)
        continue;
    end

    tr = struct();
    tr.track_id = tbl.track_id(1);
    tr.label = sprintf('id=%d', tr.track_id);
    tr.t = double(tbl.stamp_ns - tbl.stamp_ns(1)) * 1e-9;
    tr.map_x = tbl.map_x;
    tr.map_y = tbl.map_y;
    tr.base_x = tbl.base_x;
    tr.base_y = tbl.base_y;
    tr.map_vx = tbl.map_vx;
    tr.map_vy = tbl.map_vy;
    tr.base_vx = tbl.base_vx;
    tr.base_vy = tbl.base_vy;
    tr.map_speed = hypot(tbl.map_vx, tbl.map_vy);
    tr.base_speed = hypot(tbl.base_vx, tbl.base_vy);
    tr.length = tbl.length;
    tr.width = tbl.width;
    tr.confidence = tbl.confidence;
    tr.hits = tbl.hits;
    tr.missed = tbl.missed;
    tr.jump_map = [0.0; hypot(diff(tbl.map_x), diff(tbl.map_y))];
    tr.jump_base = [0.0; hypot(diff(tbl.base_x), diff(tbl.base_y))];

    dt = [0.0; double(diff(tbl.stamp_ns)) * 1e-9];
    predicted_move = [0.0; hypot(tbl.map_vx(1:end-1), tbl.map_vy(1:end-1)) .* dt(2:end)];
    tr.teleport_idx = find(tr.jump_map > max(1.5, predicted_move + 0.8));
    tr.high_jump_idx = find(tr.jump_map > 0.8);
    tr.zero_map_pose_idx = find(abs(tbl.map_x) < 1e-6 & abs(tbl.map_y) < 1e-6 & abs(tbl.map_z) < 1e-6);
    tr.zero_map_twist_idx = find(abs(tbl.map_vx) < 1e-6 & abs(tbl.map_vy) < 1e-6 & abs(tbl.map_yaw_rate_degps) < 1e-6);

    tracks{end + 1} = tr; %#ok<AGROW>
end

if isempty(tracks)
    error('No valid CSV data loaded');
end

colors = lines(numel(tracks));
fig = figure('Name', 'Tracked State Viewer', 'Color', 'w', 'Position', [80 80 1500 900]);
tl = tiledlayout(fig, 3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

ax_map = nexttile(tl, 1);
hold(ax_map, 'on'); grid(ax_map, 'on'); axis(ax_map, 'equal');
title(ax_map, 'Map Position');
xlabel(ax_map, 'map x [m]');
ylabel(ax_map, 'map y [m]');

ax_speed = nexttile(tl, 2);
hold(ax_speed, 'on'); grid(ax_speed, 'on');
title(ax_speed, 'Speed');
xlabel(ax_speed, 'time [s]');
ylabel(ax_speed, 'speed [m/s]');

ax_pose = nexttile(tl, 3);
hold(ax_pose, 'on'); grid(ax_pose, 'on');
title(ax_pose, 'Map Pose');
xlabel(ax_pose, 'time [s]');
ylabel(ax_pose, 'position [m]');

ax_size = nexttile(tl, 4);
hold(ax_size, 'on'); grid(ax_size, 'on');
title(ax_size, 'Size');
xlabel(ax_size, 'time [s]');
ylabel(ax_size, 'size [m]');

ax_jump = nexttile(tl, 5);
hold(ax_jump, 'on'); grid(ax_jump, 'on');
title(ax_jump, 'Jump Per Sample');
xlabel(ax_jump, 'time [s]');
ylabel(ax_jump, 'jump [m]');

ax_state = nexttile(tl, 6);
hold(ax_state, 'on'); grid(ax_state, 'on');
title(ax_state, 'Confidence / Missed / Hits');
xlabel(ax_state, 'time [s]');
ylabel(ax_state, 'value');

map_markers = gobjects(numel(tracks), 1);
cursor_lines = gobjects(numel(tracks), 5);

for i = 1:numel(tracks)
    tr = tracks{i};
    c = colors(i, :);

    plot(ax_map, tr.map_x, tr.map_y, '-', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s path', tr.label));
    map_markers(i) = plot(ax_map, tr.map_x(1), tr.map_y(1), 'o', ...
        'MarkerFaceColor', c, 'MarkerEdgeColor', c, 'MarkerSize', 7, ...
        'DisplayName', sprintf('%s current', tr.label));

    plot(ax_speed, tr.t, tr.map_speed, '-', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s map speed', tr.label));
    plot(ax_speed, tr.t, tr.base_speed, '--', 'Color', c, 'LineWidth', 1.0, ...
        'DisplayName', sprintf('%s base speed', tr.label));

    plot(ax_pose, tr.t, tr.map_x, '-', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s map x', tr.label));
    plot(ax_pose, tr.t, tr.map_y, '--', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s map y', tr.label));

    plot(ax_size, tr.t, tr.length, '-', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s length', tr.label));
    plot(ax_size, tr.t, tr.width, '--', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s width', tr.label));

    plot(ax_jump, tr.t, tr.jump_map, '-', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s map jump', tr.label));
    plot(ax_jump, tr.t, tr.jump_base, '--', 'Color', c, 'LineWidth', 1.0, ...
        'DisplayName', sprintf('%s base jump', tr.label));

    plot(ax_state, tr.t, tr.confidence, '-', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('%s conf', tr.label));
    plot(ax_state, tr.t, tr.missed, '--', 'Color', c, 'LineWidth', 1.0, ...
        'DisplayName', sprintf('%s missed', tr.label));
    plot(ax_state, tr.t, tr.hits, ':', 'Color', c, 'LineWidth', 1.0, ...
        'DisplayName', sprintf('%s hits', tr.label));

    if ~isempty(tr.teleport_idx)
        plot(ax_map, tr.map_x(tr.teleport_idx), tr.map_y(tr.teleport_idx), 'rp', ...
            'MarkerFaceColor', 'r', 'MarkerSize', 12, 'DisplayName', sprintf('%s teleport', tr.label));
        plot(ax_jump, tr.t(tr.teleport_idx), tr.jump_map(tr.teleport_idx), 'rp', ...
            'MarkerFaceColor', 'r', 'MarkerSize', 10, 'HandleVisibility', 'off');
    end
    if ~isempty(tr.zero_map_pose_idx)
        plot(ax_pose, tr.t(tr.zero_map_pose_idx), tr.map_x(tr.zero_map_pose_idx), 'rx', ...
            'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', sprintf('%s zero map pose', tr.label));
        plot(ax_pose, tr.t(tr.zero_map_pose_idx), tr.map_y(tr.zero_map_pose_idx), 'rx', ...
            'MarkerSize', 8, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    if ~isempty(tr.zero_map_twist_idx)
        plot(ax_speed, tr.t(tr.zero_map_twist_idx), tr.map_speed(tr.zero_map_twist_idx), 'rx', ...
            'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', sprintf('%s zero map twist', tr.label));
    end
    if ~isempty(tr.high_jump_idx)
        plot(ax_jump, tr.t(tr.high_jump_idx), tr.jump_map(tr.high_jump_idx), 'ko', ...
            'MarkerSize', 5, 'DisplayName', sprintf('%s high jump', tr.label));
    end

    cursor_lines(i, 1) = xline(ax_speed, tr.t(1), '-', 'Color', c, 'LineWidth', 1.0);
    cursor_lines(i, 2) = xline(ax_pose, tr.t(1), '-', 'Color', c, 'LineWidth', 1.0);
    cursor_lines(i, 3) = xline(ax_size, tr.t(1), '-', 'Color', c, 'LineWidth', 1.0);
    cursor_lines(i, 4) = xline(ax_jump, tr.t(1), '-', 'Color', c, 'LineWidth', 1.0);
    cursor_lines(i, 5) = xline(ax_state, tr.t(1), '-', 'Color', c, 'LineWidth', 1.0);
end

legend(ax_map, 'Location', 'best');
legend(ax_speed, 'Location', 'best');
legend(ax_pose, 'Location', 'best');
legend(ax_size, 'Location', 'best');
legend(ax_jump, 'Location', 'best');
legend(ax_state, 'Location', 'best');

summary_lines = strings(0);
for i = 1:numel(tracks)
    tr = tracks{i};
    summary_lines(end + 1) = sprintf('%s  map_speed mean=%.2f max=%.2f  jump max=%.2f  teleports=%d  zero_pose=%d  zero_twist=%d  length[%.2f, %.2f] width[%.2f, %.2f]', ...
        tr.label, mean(tr.map_speed), max(tr.map_speed), max(tr.jump_map), ...
        numel(tr.teleport_idx), numel(tr.zero_map_pose_idx), numel(tr.zero_map_twist_idx), ...
        min(tr.length), max(tr.length), min(tr.width), max(tr.width)); %#ok<AGROW>
end

annotation(fig, 'textbox', [0.08 0.94 0.88 0.05], 'String', summary_lines, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'FontSize', 10);
annotation(fig, 'textbox', [0.08 0.90 0.88 0.03], ...
    'String', 'Markers: red pentagram=teleport, red x=zero map output, black circle=high jump (>0.8m)', ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'FontSize', 9);

if playback_rate <= 0
    return;
end

t_end = max(cellfun(@(tr) tr.t(end), tracks));
last_wall = tic;
play_t = 0.0;

while ishandle(fig)
    elapsed_wall = toc(last_wall);
    last_wall = tic;
    play_t = play_t + elapsed_wall * playback_rate;
    if play_t > t_end
        play_t = t_end;
    end

    for i = 1:numel(tracks)
        tr = tracks{i};
        idx = find(tr.t <= play_t, 1, 'last');
        if isempty(idx)
            idx = 1;
        end
        set(map_markers(i), 'XData', tr.map_x(idx), 'YData', tr.map_y(idx));
        for k = 1:size(cursor_lines, 2)
            cursor_lines(i, k).Value = tr.t(idx);
        end
    end

    drawnow limitrate;
    if play_t >= t_end
        break;
    end
end
end

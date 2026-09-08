function bestYawDeg = tune_yaw_offset_from_bag(bagPath)
% Estimate imu_yaw_correction_deg from ros2 bag (/fix + /imu/data)
% Output: best yaw correction [deg] to set in obstacle_filtering.yaml

if nargin < 1 || isempty(bagPath)
    error('Pass a rosbag directory, e.g. tune_yaw_offset_from_bag("path/to/gps_test_1")');
end

br = ros2bagreader(bagPath);
fixSel = select(br, 'Topic', '/fix');
imuSel = select(br, 'Topic', '/imu/data');

fixMsgs = readMessages(fixSel);
imuMsgs = readMessages(imuSel);

if isempty(fixMsgs) || isempty(imuMsgs)
    error('No /fix or /imu/data messages found in bag: %s', bagPath);
end

% Extract fix
nFix = numel(fixMsgs);
fixT = zeros(nFix,1);
lat = zeros(nFix,1);
lon = zeros(nFix,1);
for i = 1:nFix
    m = fixMsgs{i};
    fixT(i) = double(m.header.stamp.sec) + 1e-9*double(m.header.stamp.nanosec);
    lat(i) = double(m.latitude);
    lon(i) = double(m.longitude);
end

% Convert lat/lon -> local ENU (approx) for heading from trajectory
R = 6378137.0;
lat0 = deg2rad(lat(1));
lon0 = deg2rad(lon(1));
latR = deg2rad(lat);
lonR = deg2rad(lon);
ex = (lonR - lon0) .* cos(lat0) * R;
ny = (latR - lat0) * R;

% Heading from GPS trajectory
vex = gradient(ex) ./ max(gradient(fixT), 1e-3);
vny = gradient(ny) ./ max(gradient(fixT), 1e-3);
speed = hypot(vex, vny);
gpsHeading = atan2(vny, vex);

% Filter low-speed samples
validGps = speed > 1.0;

% Extract imu yaw
nImu = numel(imuMsgs);
imuT = zeros(nImu,1);
imuYaw = zeros(nImu,1);
for i = 1:nImu
    m = imuMsgs{i};
    imuT(i) = double(m.header.stamp.sec) + 1e-9*double(m.header.stamp.nanosec);
    q = m.orientation;
    imuYaw(i) = quatToYaw(double(q.w), double(q.x), double(q.y), double(q.z));
end

% Sync imu yaw onto fix timestamps
imuYawAtFix = interp1(imuT, unwrap(imuYaw), fixT, 'linear', 'extrap');
imuYawAtFix = wrapToPi(imuYawAtFix);

candDeg = -180:0.5:180;
score = inf(size(candDeg));
for k = 1:numel(candDeg)
    c = deg2rad(candDeg(k));
    y = wrapToPi(imuYawAtFix + c);
    err = wrapToPi(gpsHeading - y);
    e = err(validGps);
    if isempty(e)
        continue;
    end
    score(k) = rms(e);
end

[~, idx] = min(score);
bestYawDeg = candDeg(idx);

fprintf('\n[Yaw tuning result]\n');
fprintf('Best imu_yaw_correction_deg = %.2f deg\n', bestYawDeg);
fprintf('RMS heading error          = %.3f rad (%.2f deg)\n', score(idx), rad2deg(score(idx)));

figure('Name','Yaw offset tuning','Color','w');
subplot(2,1,1);
plot(candDeg, rad2deg(score), 'LineWidth', 1.2); grid on;
xlabel('imu\_yaw\_correction\_deg'); ylabel('RMS heading error [deg]');
title('Yaw correction search');

subplot(2,1,2);
bestYaw = deg2rad(bestYawDeg);
plot(fixT-fixT(1), rad2deg(wrapToPi(gpsHeading)), 'k-', 'DisplayName','GPS heading'); hold on;
plot(fixT-fixT(1), rad2deg(wrapToPi(imuYawAtFix + bestYaw)), 'r-', 'DisplayName','IMU heading corrected');
grid on; xlabel('time [s]'); ylabel('heading [deg]'); legend('Location','best');
title('Heading comparison');

end

function yaw = quatToYaw(w, x, y, z)
yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
end

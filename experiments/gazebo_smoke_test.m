% Open-loop check of the MATLAB <-> Gazebo link through ros_io: start the
% sim first (gazebo/gz.sh up), then run this. Applies 10 s of equal
% thrust and 10 s of differential thrust and prints/plots the response.
clearvars
setup;
cfg = default_config(struct('preset','gazebo'));
map.start = [0;0];
io = ros_io(cfg,map);

F = [repmat([20;20],1,100), repmat([5;25],1,100)];   % [N], 10 Hz
n = size(F,2);
T = nan(1,n+1);
X = nan(6,n+1);
io.start();
for k = 1:n
    [T(k),X(:,k)] = io.read();
    io.write(F(:,k));
    io.wait();
end
[T(n+1),X(:,n+1)] = io.read();
io.stop();
rosshutdown;

dt = diff(T);
fprintf('steps %d, sim time %.1f s, step dt mean %.3f s (min %.3f, max %.3f)\n', ...
    n, T(end)-T(1), mean(dt), min(dt), max(dt));
fprintf('after straight: u = %.2f m/s, r = %.3f rad/s\n', X(4,101), X(6,101));
fprintf('after turn:     u = %.2f m/s, r = %.3f rad/s, psi = %.2f rad\n', X(4,end), X(6,end), X(3,end));
save(fullfile(cfg.resultsDir,'gazebo_smoke_test.mat'),'T','X','F');
figure; plot(X(1,:),X(2,:),'.-'); axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');

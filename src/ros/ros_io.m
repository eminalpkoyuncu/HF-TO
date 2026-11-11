function io = ros_io(cfg,map)
% ROS 1 interface to the Heron in Gazebo (uuv_simulator thruster plugin,
% topics in gazebo/README.md). Poses are returned in map coordinates:
% p_map = origin(1:2) + R(origin(3))*p_gazebo, psi_map = psi_gazebo + origin(3),
% with cfg.gz.origin = [start;0] by default. Velocities are body-frame.
origin = cfg.gz.origin;
if isempty(origin)
    origin = [map.start(1:2); 0];
end
Rot = [cos(origin(3)) -sin(origin(3)); sin(origin(3)) cos(origin(3))];

% uuv FloatStamped for MATLAB (rosgenmsg output of gazebo/matlab_msgs)
msgDir = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))),'gazebo', ...
    'matlab_msgs','matlab_msg_gen_ros1','glnxa64','install','m');
if isfolder(msgDir) && ~contains(path,'matlab_msg_gen_ros1')
    addpath(msgDir);
end

rosshutdown;
if strlength(cfg.gz.masterUri) > 0
    setenv("ROS_MASTER_URI",cfg.gz.masterUri);
    setenv("ROS_IP",cfg.gz.ip);
end
rosinit;
pub0 = rospublisher("/heron/thrusters/0/input","uuv_gazebo_ros_plugins_msgs/FloatStamped");
pub1 = rospublisher("/heron/thrusters/1/input","uuv_gazebo_ros_plugins_msgs/FloatStamped");
msg0 = rosmessage(pub0);
msg1 = rosmessage(pub1);
clockSub = rossubscriber("/clock","rosgraph_msgs/Clock");
odomSub = rossubscriber("/odometry/base");
rate = rosrate(cfg.gz.rate);

% thrust [N] -> normalized thruster command
outthrust = [-19.88 -16.52 -12.6 -5.6 -1.4 0 2.24 9.52 21.28 28.0 33.6];
intoprop = [-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1];
thrustin = @(F) min(max(interp1(outthrust,intoprop,min(max(F,outthrust(1)),outthrust(end)),'spline'),-1),1);

t0 = 0;
io.start = @start;
io.read = @read;
io.write = @write;
io.wait = @() waitfor(rate);
io.stop = @stop;

    function start()
        c = receive(clockSub,5);
        t0 = double(c.Clock_.Sec) + double(c.Clock_.Nsec)*1e-9;
        reset(rate);
    end

    function [t,x] = read()
        c = receive(clockSub,5);
        t = double(c.Clock_.Sec) + double(c.Clock_.Nsec)*1e-9 - t0;
        od = receive(odomSub,5);
        p = od.Pose.Pose.Position;
        q = od.Pose.Pose.Orientation;
        eul = quat2eul([q.W q.X q.Y q.Z]);
        tw = od.Twist.Twist;
        x = [origin(1:2) + Rot*[p.X; p.Y]; eul(1) + origin(3); tw.Linear.X; tw.Linear.Y; tw.Angular.Z];
    end

    function write(F)
        msg0.Data = thrustin(F(1));
        msg1.Data = thrustin(F(2));
        send(pub0,msg0);
        send(pub1,msg1);
    end

    function stop()
        write([0;0]);
    end
end

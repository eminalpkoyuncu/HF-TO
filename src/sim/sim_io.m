function io = sim_io(model,x0,h)
% Offline stand-in for ros_io: wait() advances the plant by h (not in real
% time). Used to dry-run gazebo_run without ROS.
x = x0(:);
t = 0;
F = [0;0];
io.start = @start;
io.read = @read;
io.write = @write;
io.wait = @wait;
io.stop = @() [];

    function start()
        t = 0;
    end

    function [tt,xx] = read()
        tt = t;
        xx = x;
    end

    function write(u)
        F = u(:);
    end

    function wait()
        if isempty(model.f)
            x = model.step(x,F);
        else
            [~,xs] = ode45(@(~,z) model.f(z,F),[0 h],x);
            x = xs(end,:).';
        end
        t = t + h;
    end
end

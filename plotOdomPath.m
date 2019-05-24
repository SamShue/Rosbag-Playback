function plotOdomPath(bag, color)

% xvec = zeros(length(msgs));
% yvec = zeros(length(msgs));
bagselect = select(bag, 'MessageType', 'nav_msgs/Odometry');
ts = timeseries(bagselect, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');
tsData = ts.Data;

if(nargin < 3)
    plot(tsData(:,1), tsData(:,2), 'Color', [.7 .7 .7]);
else
    plot(tsData(:,1), tsData(:,2), color);
end
xlabel('meters'); ylabel('meters');

end


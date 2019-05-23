function plotOdom(msg)
if(contains(msg.MessageType,'Odom'))
    hold off;
    % get pose from odom message
    position(1) = msg.Pose.Pose.Position.X;
    position(2) = msg.Pose.Pose.Position.Y;
    orientation(1) = msg.Pose.Pose.Orientation.W;
    orientation(2) = msg.Pose.Pose.Orientation.X;
    orientation(3) = msg.Pose.Pose.Orientation.Y;
    orientation(4) = msg.Pose.Pose.Orientation.Z;
    % convert quaternions to eulter angels (quat2eul format: WXYZ -> ZXY)
    orientation = quat2eul(orientation);
    % get rotation about z axis in degrees
    zOrientation = wrapTo360(rad2deg(orientation(1)));
    drawRobot(position(1), position(2), zOrientation, 0.25); hold on;
    xlim([-5 5]); ylim([-5 5]);
    xlabel('meters'); ylabel('meters');
    pause(0.001);
end
end

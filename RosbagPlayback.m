clc;
clear all;
close all;

filename = '2019-05-21-18-51-45.bag';

bag = rosbag(filename);

% get odom
bagSelect = select(bag,'Topic','/odom');
odomTs = timeseries(bagSelect, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', ... %'Twist.Twist.Angular.Z');
    'Pose.Pose.Orientation.X', 'Pose.Pose.Orientation.Y', 'Pose.Pose.Orientation.Z', 'Pose.Pose.Orientation.W');
% Convert time seriest to vector
odom = odomTs.Data; % [x, y, qx, qy, qz, qw] <- odom row contents


for ii = 1:length(odom)
    
    
    % Plot odom
    %======================================================================
    hold off;
    % convert quaternions to eulter angels (quat2eul format: WXYZ -> ZXY)
    orientation = quat2eul([odom(ii,6), odom(ii,3), odom(ii,4), odom(ii,5)]);
    zOrientation = wrapTo360(rad2deg(orientation(1)));
    drawRobot(odom(ii,1), odom(ii,2), zOrientation, 0.25); hold on;
    plot(odom(ii,1),odom(ii,2),'o');
    xlim([-5 5]); ylim([-5 5]);
    pause(0.001);
    % End plot odom
    %----------------------------------------------------------------------
end
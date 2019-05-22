clc;
clear all;
close all;

filename = '2019-05-21-18-51-45.bag';

bag = rosbag(filename);

% get odom
bagSelect = select(bag,'Topic','/odom');
odomTs = timeseries(bagSelect, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.X', 'Pose.Pose.Orientation.Y', 'Pose.Pose.Orientation.Z', 'Pose.Pose.Orientation.W');
odom = odomTs.Data;

orientation = [0, 0, 0];
for ii = 1:length(odom)
    
    hold off;
    orientation = orientation + quat2eul([odom(ii,6), odom(ii,3), odom(ii,4), odom(ii,5)]);
    drawRobot(odom(ii,1), odom(ii,2), radtodeg(orientation(3)), 0.25); hold on;
    plot(odom(ii,1),odom(ii,2),'o');
    xlim([-5 5]); ylim([-5 5]);
    pause(0.001);
end
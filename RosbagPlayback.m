clc;
clear all;
close all;

% Import custom ROS messages
%==========================================================================
% To use the custom messages, follow these steps:
% 1. Edit javaclasspath.txt, add the following file locations as new lines, and save the file:
%   C:\Users\evilr\Documents\GitHub\RosbagPlayback\custom_messages\matlab_gen\jar\pozyx_ros_examples-0.1.0.jar
% 2. Add the custom message folder to the MATLAB path by executing:
%   addpath('C:\Users\evilr\Documents\GitHub\RosbagPlayback\custom_messages\matlab_gen\msggen')
%   savepath
% 3. Restart MATLAB and verify that you can use the custom messages. 
% Type "rosmsg list" and ensure that the output contains the generated custom message types.

folderpath = 'custom_messages';
rosgenmsg(folderpath);
% End import custom ROS messages
%-------------------------------------------------------------------------

filename = '2019-05-21-18-51-45.bag';

bag = rosbag(filename);

msgs = readMessages(bag);

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
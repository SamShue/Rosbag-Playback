clc;
clear all;
close all;

% Rosbag file name
filename = '2019-05-21-18-51-45.bag';

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

% open bag file
bag = rosbag(filename);
% parse messages from bag
msgs = readMessages(bag);
msgTable = bag.MessageList;

% Rosbag playback
%==========================================================================
% iterate through all messages in bag
for ii = 1:length(msgs)
    % get timestamp of current message (not included in msg struct for some
    % reason)
    messageTimeStamp = table2array(msgTable(ii,1));
    
    
    
    % Plot odom
    %======================================================================
    % if current message is odom, plot robot position
    plotOdomMessage(msgs{ii,1});
    % End plot odom
    %----------------------------------------------------------------------
end
% End rosbag playback
%--------------------------------------------------------------------------

% Plot results
%==========================================================================
plotOdomPath(bag);
plotTwistPath(bag);
% End plot results
%--------------------------------------------------------------------------
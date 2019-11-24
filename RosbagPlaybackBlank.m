clc;
clear all;
close all;

% Rosbag file name
filename = '2019-05-21-18-56-11.bag';

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
timeStamps = table2array(msgTable(:,1));

% Rosbag playback
%==========================================================================
% iterate through all messages in bag
for ii = 1:length(msgs) 
    if(contains(msgs{ii,1}.MessageType,'Odom'))
        odomPose = parseOdomMessage2D(msgs{ii,1});
    end
    
    % Render environment
    %======================================================================
    %     if(mod(ii, 500) == 0) % render every 100 messages
    %         clf;
    %         hold on;
    %         title(sprintf('Iteration %d', ii));
    %         xlim([-5 5]); ylim([-5 5]);
    %         xlabel('meters'); ylabel('meters');
    %         drawRobot(odomPose(1), odomPose(2), odomPose(3), 0.25);
    %         if(~isempty(node))
    %             for jj = 1:length(node)
    %                 node(jj).plotParticles();
    %             end
    %         end
    %         pause(0.001);
    %     end
    % End render environment
    %----------------------------------------------------------------------
end
% End rosbag playback
%--------------------------------------------------------------------------

% Plot results
%==========================================================================
hold on;
xlim([-5 5]); ylim([-5 5]);
xlabel('meters'); ylabel('meters');
drawRobot(odomPose(1), odomPose(2), odomPose(3), 0.25);

plotOdomPath(bag);
plotTwistPath(bag);
% End plot results
%--------------------------------------------------------------------------
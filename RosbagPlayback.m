clc;
clear all;
close all;

% Rosbag file name
% filename = 'rosbags/2019-05-21/2019-05-21-18-51-45.bag';
filename = 'rosbags/2019-06-04/2019-06-04-16-26-32.bag';

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

numParticles = 100;
odomPose = [0,0,0];
node = [];
robot = robotpf(numParticles, 0, 0, 0);

% Rosbag playback
%==========================================================================
% iterate through all messages in bag
for ii = 1:length(msgs)
    % get timestamp of current message (not included in msg struct for some
    % reason)
    messageTimeStamp = table2array(msgTable(ii,1));
    
    if(contains(msgs{ii,1}.MessageType, 'DeviceRange'))
        % convert range from mm to m
        dist_m = double(msgs{ii,1}.Distance)/1000;
        if(isempty(node))
            node = [node, nodepf(msgs{ii,1}.Device, numParticles, odomPose(1), odomPose(2), dist_m)];
        else
            found = 0;
            for jj = 1:length(node)
                if(strcmp(node(jj).addr, msgs{ii,1}.Device))
                    % Update existing pf
                    node(jj).resample(odomPose(1), odomPose(2), double(msgs{ii,1}.Distance)/1000);
                    found = 1;
                end
            end
            
            if(found == 0)
                % no matching addr found, append new pf
                node = [node, nodepf(msgs{ii,1}.Device, numParticles, odomPose(1), odomPose(2), dist_m)];
            end
        end
    end
    
    if(contains(msgs{ii,1}.MessageType,'Twist'))
        
    end
    
    if(contains(msgs{ii,1}.MessageType,'Odom'))
        % get pose from odom message
        position(1) = msgs{ii,1}.Pose.Pose.Position.X;
        position(2) = msgs{ii,1}.Pose.Pose.Position.Y;
        orientation(1) = msgs{ii,1}.Pose.Pose.Orientation.W;
        orientation(2) = msgs{ii,1}.Pose.Pose.Orientation.X;
        orientation(3) = msgs{ii,1}.Pose.Pose.Orientation.Y;
        orientation(4) = msgs{ii,1}.Pose.Pose.Orientation.Z;
        % convert quaternions to eulter angels (quat2eul format: WXYZ -> ZXY)
        orientation = quat2eul(orientation);
        odomPose = [position(1), position(2), wrapTo360(rad2deg(orientation(1)))];
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
if(~isempty(node))
    for jj = 1:length(node)
        node(jj).plotParticles();
    end
end

plotOdomPath(bag);
plotTwistPath(bag);
% End plot results
%--------------------------------------------------------------------------
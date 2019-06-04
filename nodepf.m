classdef nodepf < handle
    %NODEPF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticles;
        particles;
        color = 'green';
        addr;
        minResampleDistance = 0.25;
        lastMeasurement;
        nodeStdDev_m = 0.5;
    end
    
    methods
        function obj = nodepf(addr, numParticles, robotPosX, robotPosY, range_m)
            %NODEPF Construct an instance of this class
            %   Detailed explanation goes here
            obj.addr = addr;
            if(strcmp(addr, '26965'))
                obj.color = 'blue';
            elseif(strcmp(addr, '26933'))
                obj.color = 'red';
            elseif(strcmp(addr, '26935'))
                obj.color = 'yellow';
            end
            
            obj.numParticles = numParticles;
            
            particles = zeros(numParticles, 2);
            % Generate Particles
            for ii = 1:numParticles
                orientation = rand() * 360;
                noise = normrnd(0, obj.nodeStdDev_m);
                x = cosd(orientation)*(range_m + noise);
                y = sind(orientation)*(range_m + noise);
                particles(ii,:) = [x, y];
            end
            
            obj.particles = particles;
            obj.lastMeasurement = range_m;
        end
        
        function predict(obj, motion)
            % Empty function; nodes do not move, nothing to update
        end
        
        function resample(obj, robotPosX, robotPosY, range_m)
            % Check to see if robot has moved enough to warrant resampling
            if(abs(obj.lastMeasurement - range_m) > obj.minResampleDistance)
                % Get expected range values for each particle
%                 err = zeros(length(obj.numParticles));
%                 for ii = 1:obj.numParticles
                    x = obj.particles(:,1) - robotPosX;
                    y = obj.particles(:,2) - robotPosY;
                    expectedRange = sqrt(x.^2 + y.^2);
                    err = abs(expectedRange - range_m);
%                 end
                
                norm_err = abs(err - max(err));
                weights = norm_err./sum(norm_err);
                
                newParticles = zeros(obj.numParticles, 2);
                for ii = 1:obj.numParticles
                    newParticles(ii,:) = obj.particles(RandFromDist(weights),:);
                    % Add noise to resampled particles
                    newParticles(ii,1) = newParticles(ii,1) + normrnd(0, 0.1);
                    newParticles(ii,2) = newParticles(ii,2) + normrnd(0, 0.1);
                end
                
                % Replace old particle set
                obj.particles = newParticles;
                % Update last robot position
                obj.lastMeasurement = range_m;
            end
        end
        
        function plotParticles(obj)
            hold on;
            for ii = 1:obj.numParticles
                scatter(obj.particles(ii,1), obj.particles(ii,2), 'o', obj.color);
            end
        end
    end
end


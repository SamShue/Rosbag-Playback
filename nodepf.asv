classdef nodepf < handle
    %NODEPF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticles;
        particles;
        color = 'red';
        addr;
        
        nodeStdDev_m = 0.5;
    end
    
    methods
        function obj = nodepf(addr, numParticles, robotPosX, robotPosY, range_m)
            %NODEPF Construct an instance of this class
            %   Detailed explanation goes here
            obj.addr = addr;
            
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
        end
        
        function outputArg = predict(obj, motion)
            
        end
        
        function outputArg = resample(obj, robotPosX, robotPosY, range_m)
            % Get expected range values for each particle
            err = zeros(length(obj.numParticles));
            for ii = 1:obj.numParticles
                x = obj.particles(ii,1) - robotPosX;
                y = obj.particles(ii,2) - robotPosY;
                expectedRange = sqrt(x^2 + y^2);
                err(ii) = abs(expectedRange - range_m);
            end
            
            norm_err = abs(err - max(err));
            weights = norm_err./sum(norm_err);
            
            newParticles = zeros(obj.numParticles, 2);
            for ii = 1:obj.numParticles
                newParticles(ii,:) = obj.particles(RandFromDist(weights),:);
            end
            
            % Replace old particle set
            obj.particles = newParticles;
        end
        
        function plotParticles(obj)
            hold on;
            for ii = 1:obj.numParticles
                scatter(obj.particles(ii,1), obj.particles(ii,2), 'o', obj.color);
            end
        end
    end
end


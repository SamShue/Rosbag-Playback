classdef nodepf
    %NODEPF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticles;
        particles;
        color = 'red';
        
        nodeStdDev_m = 1;
    end
    
    methods
        function obj = nodepf(numParticles, robotPosX, robotPosY, range_m)
            %NODEPF Construct an instance of this class
            %   Detailed explanation goes here
            obj.numParticles = numParticles;
            
            particles = zeros(numParticles, 2);
            % Generate Particles
            for ii = 1:numParticles
                orientation = rand() * 360;
                noise = normrnd(0, nodeStdDev_m);
                x = cosd(orientation)*(range_m + noise);
                y = sind(orientation)*(range_m + noise);
                particles(ii,:) = [x, y];
            end
            
            obj.particles = particles;
        end
        
        function outputArg = predict(obj, motion)
            
        end
        
        function outputArg = resample(obj, robotPosX, robotPosY, range_m)
            
        end
        
        function plotParticles(obj)
            hold on;
            for ii = 1:obj.numParticles
                scatter(obj.particles(ii,1), obj.particles(ii,2), 'o', obj.color);
            end
        end
    end
end


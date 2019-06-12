classdef robotpf < handle
    %ROBOTPF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticles;
        particles;
        lastMeasurement;
        linearNoise = 0.001;
        angularNoise = 0.001;
    end
    
    methods
        function obj = robotpf(numParticles, robotPosX, robotPosY, robotOrientation)
            obj.numParticles = numParticles;
            obj.particles = zeros(numParticles, 3);
            for ii = 1:numParticles
                obj.particles(ii,:) = [robotPosX, robotPosY, robotOrientation];
            end
            
        end
        
        function predict(obj, v, w, d_t)
            for ii = 1:obj.numParticles
                theta = obj.particles(ii,3);
                r = v/w;
                
                s = sin(theta);
                c = cos(theta);
                s_th = sin(theta+w*d_t);
                c_th = cos(theta+w*d_t);
                
                n_l = normrnd(0, obj.linearNoise);
                n_w = normrnd(0, obj.angularNoise);
                
                if(w<.05)
                    %basically robot is going straight
                    obj.particles(ii,1) = obj.particles(ii,1) + (v*c_th)*d_t + n_l;
                    obj.particles(ii,2) = obj.particles(ii,2) + (v*s_th)*d_t + n_l;
                    obj.particles(ii,3) = obj.particles(ii,3) + w*d_t + n_w;
                else
                    %robot is turning
                    obj.particles(ii,1) = obj.particles(ii,1) + (-r*s)+(r*s_th) + n_l;
                    obj.particles(ii,2) = obj.particles(ii,2) +( r*c)-(r*c_th) + n_l;
                    obj.particles(ii,3) = obj.particles(ii,3) + w*d_t + n_w;
                end
            end
        end
        
        function resample(obj, landmarkRanges, landmarkPositions)
            err = zeros(length(obj.particles),1);
            for ii = 1:landmarkRanges
                % Get expected range values for each particle
                x = obj.particles(:,1) - landmarkPositions(ii,1);
                y = obj.particles(:,2) - landmarkPositions(ii,2);
                expectedRange = sqrt(x.^2 + y.^2);
                err = err + abs(expectedRange - landmarkRanges);
            end
            
            norm_err = abs(err - max(err));
            weights = norm_err./sum(norm_err);
            
            newParticles = zeros(obj.numParticles, 2);
            for ii = 1:obj.numParticles
                newParticles(ii,:) = obj.particles(RandFromDist(weights),:);
                % Add noise to resampled particles
                newParticles(ii,1) = newParticles(ii,1) + normrnd(0, 0.025);
                newParticles(ii,2) = newParticles(ii,2) + normrnd(0, 0.025);
            end
            
            % Replace old particle set
            obj.particles = newParticles;
        end
        
        function plotParticles(obj)
            hold on;
            for ii = 1:obj.numParticles
                scatter(obj.particles(ii,1), obj.particles(ii,2), 'o', 'black');
            end
        end
        
    end
end


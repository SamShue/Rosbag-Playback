classdef robotmappf < handle
    %ROBOTPF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticles;
        particles;
        
        linearNoise = 0.05;
        angularNoise = 0.1;
        measurementNoise = 0.001;
        
        landmarkIds = [];
        lastMeasurement;
    end
    
    methods
        function obj = robotmappf(numParticles, robotPosX, robotPosY, robotOrientation)
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
                    obj.particles(ii,3) = obj.particles(ii,3) + w*d_t;
                else
                    %robot is turning
                    obj.particles(ii,1) = obj.particles(ii,1) + (-r*s)+(r*s_th) + n_l;
                    obj.particles(ii,2) = obj.particles(ii,2) +( r*c)-(r*c_th) + n_l;
                    obj.particles(ii,3) = obj.particles(ii,3) + w*d_t + n_w;
                end
            end
        end
        
        function resample(obj, landmarkRanges, landmarkId)
            err = zeros(length(obj.particles),1);
            
            for ii = 1:length(landmarkRanges)
                if(landmarkRanges(ii) ~= 0)
                    lidx = find(obj.landmarkIds == landmarkId(ii));
                    
                    lidx = ((lidx - 1)*2 + 1) + 3; % find landmark x offset in state vector
                    % Get expected range values for each particle
                    x = obj.particles(:,1) - obj.particles(:,lidx);
                    y = obj.particles(:,2) - obj.particles(:,lidx + 1);
                    expectedRange = sqrt(x.^2 + y.^2);
                    err = err + abs(expectedRange - landmarkRanges(ii));
                end
            end
            
            norm_err = abs(err - max(err));
            weights = norm_err./sum(norm_err);
            
            newParticles = zeros(size(obj.particles));
            for ii = 1:obj.numParticles
                newParticles(ii,:) = obj.particles(RandFromDist(weights),:);
                % Add noise to resampled particles
                newParticles(ii,:) = newParticles(ii,:) + [0,0,0,normrnd(0, obj.measurementNoise,[1, 2*length(obj.landmarkIds)])];
            end
            
            % Replace old particle set
            obj.particles = newParticles;
            
        end
        
        function pos = getPosition(obj)
            pos = mean(obj.particles(:,1:2));
        end
        
        function addLandmarkParticles(obj, landmarkPositions, landmarkId)
            % if landmark not appended, add particles to state vector
            if(~obj.findLandmark(landmarkId))
                obj.landmarkIds = [obj.landmarkIds; landmarkId];
                obj.particles = [obj.particles, landmarkPositions];
            end
        end
        
        function idx = findLandmark(obj, addr)
            if(isempty(obj.landmarkIds))
                idx = 0;
            else
                idx = find(obj.landmarkIds == addr);
                if(isempty(idx))
                    idx = 0;
                end
            end
        end
        
        function plotParticles(obj)
            hold on;
            for ii = 1:obj.numParticles
                scatter(obj.particles(ii,1), obj.particles(ii,2), 'o', 'black');
                for jj = 1:length(obj.landmarkIds)
                    lidx = ((jj - 1)*2 + 1) + 3;
                    switch obj.landmarkIds(jj)
                        case 26965
                            color = 'blue';
                        case 26933
                            color = 'red';
                        case 26935
                            color = 'yellow';
                        case 28162
                            color = 'magenta';
                        case 27006
                            color = 'cyan';
                        case 26944
                            color = 'green';
                        otherwise
                            color = 'green';
                    end
                    scatter(obj.particles(ii,lidx), obj.particles(ii, lidx + 1), color);
                end
            end
        end
    end
end


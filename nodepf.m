classdef nodepf
    %NODEPF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticles;
        particles;
    end
    
    methods
        function obj = nodepf(numParticles, range_m)
            %NODEPF Construct an instance of this class
            %   Detailed explanation goes here
            obj.numParticles = numParticles;
        end
        
        function outputArg = predict(obj, motion)
            
        end
        
        function outputArg = resample(obj, motion)
            
        end
    end
end


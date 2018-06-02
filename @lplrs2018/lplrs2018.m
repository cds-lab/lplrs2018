classdef lplrs2018 < handle
    %lplrs2018 class for communication with Arduino running the lplrs2018
    %sketch to control a reward system.
    %
    %  Lee Lovejoy
    %  June 2018
    %  ll2833@columbia.edu
    
    properties (SetAccess=private)
        serialObj
        
        portName = '/dev/ttyACM0';
        baud = 115200;
    end
    
    methods
        
        %  Class constructor        
        function obj = lplrs2018(varargin)
            
            %  Update parameter values
            for i=1:2:nargin
                if(any(strcmp(properties(mfilename),varargin{i})))
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
            
            obj.serialObj = serial(obj.portName,'baud',obj.baud);
            fopen(obj.serialObj);

            %  Opening the port will restart the Arduino and cause it to
            %  dump some text to the serial port, so wait a moment for this
            %  to complete.
            while(obj.serialObj.BytesAvailable==0)
            end
            
            %  Clear the serial buffer and write contents to the command
            %  line
            while(obj.serialObj.BytesAvailable > 0)
                temp = fscanf(obj.serialObj);
                fprintf('%s',temp(1:end-1));
            end
            
        end
        
        %  Class destructor        
        function delete(obj)
            obj.close;
        end
        
        %  Close connection
        function close(obj)
            fclose(obj.serialObj);
        end
        
        %  Send command to tare scale
        function tare(obj)
            fprintf(obj.serialObj,'T');
        end
        
        %  Send command to measure weight (in grams)
        function measureWeight(obj)
            
            %  Prior to sending command to measure weight, clear the
            %  incoming serial buffer.
            while(obj.serialObj.BytesAvailable>0)
                fscanf(obj.serialObj);
            end
            
            %  Write 'U' to serial
            fprintf(obj.serialObj,'U');
        end
        
        %  Read weight out of serial buffer
        function output = readWeight(obj)
            output = fscanf(obj.serialObj);
        end
    end
end


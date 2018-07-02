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
        
        suppressOutput = false;
        
        mostRecentRead = 0;
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
            
            %  Close the port if it is already open
            s = instrfind('Port',obj.portName,'Status','open');
            if(~isempty(s))
                fclose(s);
            end
            
            %  Create the serial object and open it
            obj.serialObj = serial(obj.portName,'baud',obj.baud);
            fopen(obj.serialObj);
            
            %  Opening the port will restart the Arduino and cause it to
            %  dump some text to the serial port, so wait a moment for this
            %  to complete.
            while(obj.serialObj.BytesAvailable==0)
            end
            
            %  Write greeting to command window (first line)
            temp = fscanf(obj.serialObj);
            fprintf('%s',temp(1:end-1));
            
            %  Clear the serial buffer and write contents to the command
            %  line unless this is suppressed
            while(obj.serialObj.BytesAvailable > 0)
                temp = fscanf(obj.serialObj);
                if(~obj.suppressOutput);
                    fprintf('%s',temp(1:end-1));
                end
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
        
        %  Send command to serial buffer
        function writeSerial(obj,argument)
            fprintf(obj.serialObj,sprintf('%s',argument));
        end
        
        %  Clear serial buffer
        function clearSerialBuffer(obj)
            while(obj.serialObj.BytesAvailable>0)
                fscanf(obj.serialObj);
            end
        end
        
        %  Read serial data if available in serial buffer
        %  Include optional argument 'wait' if you want to wait for data
        function readSerial(obj,varargin)
            if(any(strcmpi('wait',varargin)))
                while(obj.serialObj.BytesAvailable==0)
                end
            end
            if(obj.serialObj.BytesAvailable>0)
                obj.mostRecentRead = fscanf(obj.serialObj,'%f');
            end
        end
        
        %  Send the command to tare the scale
        function startTare(obj)            
            obj.writeSerial('T10');
        end
        
        %  Send the command to read the weight
        function startWeigh(obj)
            obj.writeSerial('U10');
        end
    end
end


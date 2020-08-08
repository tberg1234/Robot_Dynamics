classdef points_to_go_to < ros.Message
    %points_to_go_to MATLAB implementation of robot_with_vision/points_to_go_to
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'robot_with_vision/points_to_go_to' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '5adf25fa7bb7cc8944f6c9a793848bf8' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        StdMsgsHeaderClass = ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        XPoints
        YPoints
        ZPoints
    end
    
    properties (Access = protected)
        Cache = struct('Header', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Header', 'XPoints', 'YPoints', 'ZPoints'} % List of non-constant message properties
        ROSPropertyList = {'header', 'x_points', 'y_points', 'z_points'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = points_to_go_to(msg)
            %points_to_go_to Construct the message object points_to_go_to
            import com.mathworks.toolbox.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('ros:mlros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('ros:mlros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('ros:mlros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'points_to_go_to', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function xpoints = get.XPoints(obj)
            %get.XPoints Get the value for property XPoints
            javaArray = obj.JavaMessage.getXPoints;
            array = obj.readJavaArray(javaArray, 'char');
            xpoints = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.XPoints(obj, xpoints)
            %set.XPoints Set the value for property XPoints
            if isstring(xpoints)
                xpoints = cellstr(xpoints);
            end
            
            if ~isvector(xpoints) && isempty(xpoints)
                % Allow empty [] input
                xpoints = cell.empty(0,1);
            end
            
            validateattributes(xpoints, {'cell', 'string'}, {'vector'}, 'points_to_go_to', 'XPoints');
            if any(cellfun(@(x) ~ischar(x), xpoints))
                error(message('ros:mlros:message:CellArrayStringError', ...
                    'xpoints'));
            end
            
            javaArray = obj.JavaMessage.getXPoints;
            array = obj.writeJavaArray(xpoints, javaArray, 'char');
            obj.JavaMessage.setXPoints(array);
        end
        
        function ypoints = get.YPoints(obj)
            %get.YPoints Get the value for property YPoints
            javaArray = obj.JavaMessage.getYPoints;
            array = obj.readJavaArray(javaArray, 'char');
            ypoints = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.YPoints(obj, ypoints)
            %set.YPoints Set the value for property YPoints
            if isstring(ypoints)
                ypoints = cellstr(ypoints);
            end
            
            if ~isvector(ypoints) && isempty(ypoints)
                % Allow empty [] input
                ypoints = cell.empty(0,1);
            end
            
            validateattributes(ypoints, {'cell', 'string'}, {'vector'}, 'points_to_go_to', 'YPoints');
            if any(cellfun(@(x) ~ischar(x), ypoints))
                error(message('ros:mlros:message:CellArrayStringError', ...
                    'ypoints'));
            end
            
            javaArray = obj.JavaMessage.getYPoints;
            array = obj.writeJavaArray(ypoints, javaArray, 'char');
            obj.JavaMessage.setYPoints(array);
        end
        
        function zpoints = get.ZPoints(obj)
            %get.ZPoints Get the value for property ZPoints
            javaArray = obj.JavaMessage.getZPoints;
            array = obj.readJavaArray(javaArray, 'char');
            zpoints = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.ZPoints(obj, zpoints)
            %set.ZPoints Set the value for property ZPoints
            if isstring(zpoints)
                zpoints = cellstr(zpoints);
            end
            
            if ~isvector(zpoints) && isempty(zpoints)
                % Allow empty [] input
                zpoints = cell.empty(0,1);
            end
            
            validateattributes(zpoints, {'cell', 'string'}, {'vector'}, 'points_to_go_to', 'ZPoints');
            if any(cellfun(@(x) ~ischar(x), zpoints))
                error(message('ros:mlros:message:CellArrayStringError', ...
                    'zpoints'));
            end
            
            javaArray = obj.JavaMessage.getZPoints;
            array = obj.writeJavaArray(zpoints, javaArray, 'char');
            obj.JavaMessage.setZPoints(array);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.XPoints = obj.XPoints;
            cpObj.YPoints = obj.YPoints;
            cpObj.ZPoints = obj.ZPoints;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.XPoints = strObj.XPoints;
            obj.YPoints = strObj.YPoints;
            obj.ZPoints = strObj.ZPoints;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
        end
    end
    
    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.XPoints = obj.XPoints;
            strObj.YPoints = obj.YPoints;
            strObj.ZPoints = obj.ZPoints;
            strObj.Header = saveobj(obj.Header);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.robot_with_vision.points_to_go_to.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.robot_with_vision.points_to_go_to;
            obj.reload(strObj);
        end
    end
end

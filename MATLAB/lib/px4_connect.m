function client = px4_connect(ip_address, port)
%PX4_CONNECT Establish TCP connection to PX4-ROS2 bridge
%   Creates and returns a TCP client connection to the specified IP and port
%
%   Input:
%       ip_address - IP address of the PX4-ROS2 bridge server
%       port       - Port number for connection
%
%   Output:
%       client     - TCP client object

    try
        disp(['Connecting to ' ip_address ':' num2str(port) '...']);
        client = tcpclient(ip_address, port, 'Timeout', 10);
        disp(['Connected to ' ip_address ':' num2str(port)]);
    catch e
        error(['Failed to connect: ' e.message]);
    end
end


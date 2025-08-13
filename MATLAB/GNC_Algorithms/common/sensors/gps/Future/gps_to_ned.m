function [north, east, down] = gps_to_ned(gps_lat, gps_lon, gps_alt, config, client)
%GPS_TO_NED Convert GPS coordinates to local NED coordinates
%   Converts GPS latitude, longitude, altitude to North-East-Down coordinates
%   relative to the home position set in config.
%
%   Inputs:
%       gps_lat  - GPS latitude in degrees
%       gps_lon  - GPS longitude in degrees  
%       gps_alt  - GPS altitude in meters (MSL)
%       config   - Configuration structure with home position
%       client   - TCP client for validation (optional)
%
%   Outputs:
%       north    - North coordinate in meters
%       east     - East coordinate in meters
%       down     - Down coordinate in meters (negative up)
%
%   Usage:
%       [n, e, d] = gps_to_ned(47.398, 8.546, 100.5, config)
%       [n, e, d] = gps_to_ned(lat, lon, alt, config, client) % With validation

    % Basic input validation
    validate_conversion_inputs(gps_lat, gps_lon, gps_alt, config, nargin);
    
    % Extract home reference
    home_lat = config.gps.home_position.latitude_deg;
    home_lon = config.gps.home_position.longitude_deg;
    home_alt = config.gps.home_position.altitude_msl_m;
    
    % Perform GPS→NED conversion using available toolbox
    [north, east, down] = perform_gps_conversion(gps_lat, gps_lon, gps_alt, ...
                                               home_lat, home_lon, home_alt);
    
    % Optional cross-validation against PX4
    if nargin >= 5 && ~isempty(client) && should_validate(config)
        px4_validate_gps_ned([north, east, down], client, config);
    end
end
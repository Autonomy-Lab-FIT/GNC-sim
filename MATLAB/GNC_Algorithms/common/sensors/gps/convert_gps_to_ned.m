function [north, east, down] = convert_gps_to_ned(telemetry_data, gps_reference)
%CONVERT_GPS_TO_NED Convert current GPS reading to NED coordinates
%   Use this function in your main loop to convert GPS data
%
%   Inputs:
%       telemetry_data - Current telemetry structure
%       gps_reference - Reference point from setup_gps_reference
%
%   Outputs:
%       north, east, down - NED coordinates in meters

    if ~gps_reference.reference_set
        error('GPS reference not set. Call setup_gps_reference first.');
    end
    
    % Extract current GPS data
    if isfield(telemetry_data, 'gps') && ~isempty(telemetry_data.gps)
        gps = telemetry_data.gps;
        
        % Convert to NED
        [north, east, down] = geodetic_to_ned(gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m, ...
                                              gps_reference.lat_ref, ...
                                              gps_reference.lon_ref, ...
                                              gps_reference.alt_ref);
    else
        % Return zeros if no GPS data available
        north = 0;
        east = 0;
        down = 0;
        warning('No GPS data available, returning zeros');
    end
end
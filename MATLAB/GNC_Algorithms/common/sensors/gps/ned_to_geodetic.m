function [lat, lon, alt] = ned_to_geodetic(north, east, down, lat_ref, lon_ref, alt_ref)
%NED_TO_GEODETIC Convert NED coordinates back to GPS
%   Inputs:
%       north, east, down - NED coordinates in meters
%       lat_ref, lon_ref, alt_ref - Reference position (degrees, degrees, meters)
%   Outputs:
%       lat, lon, alt - GPS coordinates (degrees, degrees, meters)

    [lat, lon, alt] = ned2geodetic(north, east, down, lat_ref, lon_ref, alt_ref, 'WGS84');
end
function [north, east, down] = geodetic_to_ned(lat, lon, alt, lat_ref, lon_ref, alt_ref)
%GEODETIC_TO_NED Convert GPS coordinates to NED (North-East-Down) frame
%   Inputs:
%       lat, lon, alt    - Current GPS position (degrees, degrees, meters)
%       lat_ref, lon_ref, alt_ref - Reference position (degrees, degrees, meters)
%   Outputs:
%       north, east, down - NED coordinates in meters
%
    wgs84 = wgs84Ellipsoid('meters');
    [north, east, down] = geodetic2ned(lat, lon, alt, lat_ref, lon_ref, alt_ref, wgs84);
    
end
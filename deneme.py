function latlonalt = ned_to_latlonalt(ned_coords, ref_lat, ref_lon, ref_alt)
    % Converts NED frame coordinates to latitude, longitude, and altitude coordinates.

    % Convert reference latitude and longitude to radians
    ref_lat_rad = deg2rad(ref_lat);
    ref_lon_rad = deg2rad(ref_lon);

    % Define the WGS84 ellipsoid parameters
    a = 6378137.0;  % semi-major axis
    f = 1 / 298.257223563;  % flattening

    % Calculate the radius of curvature in the prime vertical
    e2 = f * (2 - f);  % eccentricity squared
    N = a / sqrt(1 - e2 * sin(ref_lat_rad)^2);  % radius of curvature in the prime vertical

    % Convert NED coordinates to ECEF coordinates
    north = ned_coords(1);
    east = ned_coords(2);
    down = ned_coords(3);
    x = -north;
    y = east;
    z = -down;

    % Calculate the ECEF coordinates
    ecef_x = x * sin(ref_lat_rad) * cos(ref_lon_rad) + y * sin(ref_lat_rad) * sin(ref_lon_rad) - z * cos(ref_lat_rad);
    ecef_y = x * sin(ref_lon_rad) - y * cos(ref_lon_rad);
    ecef_z = x * cos(ref_lat_rad) * cos(ref_lon_rad) + y * cos(ref_lat_rad) * sin(ref_lon_rad) + z * sin(ref_lat_rad);

    % Convert ECEF coordinates to latitude, longitude, and altitude
    p = sqrt(ecef_x^2 + ecef_y^2);
    lon = atan2(ecef_y, ecef_x);
    lat = atan2(ecef_z, p);
    N_phi = a / sqrt(1 - e2 * sin(lat)^2);
    alt = p / cos(lat) - N_phi;

    % Convert latitude and longitude to degrees
    lat_deg = rad2deg(lat);
    lon_deg = rad2deg(lon);

    % Return the coordinates as an array
    latlonalt = [lat_deg, lon_deg, alt + ref_alt];
end

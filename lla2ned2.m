function lla = lla2ned2(xyzNED,lla0)

% xyzNED = [x(1) x(2) x(3)];
% lla0 = [40 5 8000];

earth_radius = 6378137;  % radius of the earth in meters
north = xyzNED(1);
east = xyzNED(2);
lat_offset = north / earth_radius * (180 / pi);
lon_offset = east / earth_radius * (180 / pi) / cos(lla0(1) * pi / 180);

% Calculate new latitude and longitude
lat = lla0(1) + lat_offset;
lon = lla0(2) + lon_offset;

lla = [lat lon];

end
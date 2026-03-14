from math import radians, sin, cos, sqrt, atan2, pi
from models import Target

class Calculations:

    def calculate_distance(drone1_lat, drone1_lon, drone2_lat, drone2_lon, drone1_alt, drone2_alt):
        # Convert latitude and longitude from degrees to radians
        drone1_lat = radians(drone1_lat)
        drone1_lon = radians(drone1_lon)
        drone2_lat = radians(drone2_lat)
        drone2_lon = radians(drone2_lon)

        # Earth radius in meters
        R = 6371000.0  

        # Differences in latitudes and longitudes
        dlat = drone2_lat - drone1_lat
        dlon = drone2_lon - drone1_lon

        # Haversine formula to calculate distance between drones
        a = sin(dlat / 2)**2 + cos(drone1_lat) * cos(drone2_lat) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        # Distance between drones
        distance_between_drones = R * c

         # Altitude difference
        altitude_diff = drone2_alt - drone1_alt

           # Pythagorean theorem to calculate 3D distance
        distance_3d = sqrt(distance_between_drones**2 + altitude_diff**2)

        return distance_3d


    def calculate_target_position(drone1_angle, drone2_angle, distance_between_drones, drone1_altitude, drone2_altitude):
     # Convert angles to radians
        drone1_angle_rad = radians(drone1_angle)
        drone2_angle_rad = radians(drone2_angle)
    
    # Calculate target altitude
        target_altitude = (drone1_altitude + drone2_altitude) / 2
    
    # Calculate distance from each drone to target
        d1 = distance_between_drones * sin(drone2_angle_rad) / sin(pi - drone1_angle_rad - drone2_angle_rad)
        d2 = distance_between_drones * sin(drone1_angle_rad) / sin(pi - drone1_angle_rad - drone2_angle_rad)
    
    # Convert distances to coordinates in longitude and latitude
    # Assumption: 1 degree of longitude = 111 km, 1 degree of latitude = 111 km
        lon_per_meter = 1 / (111 * cos((drone1_angle + drone2_angle) / 2))
        lat_per_meter = 1 / 111
    
    # Calculate coordinates of target
        target_lon1 = d1 * cos(drone1_angle_rad) * lon_per_meter
        target_lat1 = d1 * sin(drone1_angle_rad) * lat_per_meter
        target_lon2 = d2 * cos(drone2_angle_rad) * lon_per_meter
        target_lat2 = d2 * sin(drone2_angle_rad) * lat_per_meter
    
        return Target(target_lon1, target_lat1, target_lon2, target_lat2, target_altitude)

# Example data
drone1_angle = 30  # Angle in degrees
drone2_angle = 45  # Angle in degrees
drone1_altitude = 50  # Altitude of drone 1 in meters
drone2_altitude = 60  # Altitude of drone 2 in meters

drone1_lat = 37.7749
drone1_lon = -122.4194

drone2_lat = 37.7751
drone2_lon = -122.4193

# Calculate distance to target
distance_between_drones = Calculations.calculate_distance(drone1_lat, drone1_lon, drone2_lat, drone2_lon, drone1_altitude, drone2_altitude)
print("Distance between drones:", distance_between_drones, "meters")

# Calculate target position
target_position = Calculations.calculate_target_position(drone1_angle, drone2_angle, distance_between_drones, drone1_altitude, drone2_altitude)
print("Target position:", target_position)
#!/usr/bin/env python3

# Predefined locations in the map frame (in meters)
# Coordinates below are EXAMPLES for turtlebot3_world.launch map.
# Use RViz ('Publish Point' tool or '2D Nav Goal' echo) to get coordinates for your map.
LOCATIONS = {
    "CENTER":         {"x":  0.0, "y":  0.0, "yaw": 0.0},      # Origin
    "POINT_A":        {"x":  1.5, "y":  1.0, "yaw": 90.0},     # Example top right-ish
    "POINT_B":        {"x": -1.0, "y":  1.5, "yaw": 180.0},    # Example top left-ish
    "POINT_C":        {"x": -1.5, "y": -1.0, "yaw": -90.0},    # Example bottom left-ish
    "POINT_D":        {"x":  1.0, "y": -1.5, "yaw": 0.0},      # Example bottom right-ish
    "L1":             {"x":  0.888, "y": 1.930, "yaw": 0.0},   # Your example point
    "L2":             {"x": -0.5, "y": -1.8, "yaw": 90.0},    # Another example
    # Add more meaningful names and coordinates
}

def get_location_coords(name):
    """Looks up location coordinates by name (case-insensitive)."""
    # Strip leading/trailing whitespace and convert to uppercase for reliable matching
    clean_name = name.strip().upper()
    return LOCATIONS.get(clean_name)

def get_known_locations():
    """Returns a list of known location names."""
    return list(LOCATIONS.keys())
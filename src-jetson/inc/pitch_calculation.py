import math

def pixels_to_degrees(pixel_x, pixel_y):
    # Camera resolution
    resolution_x = 3280
    resolution_y = 2464
    
    # Field of view in degrees
    fov_degrees = 160
    
    # Calculate the angle of view in radians
    fov_radians = math.radians(fov_degrees)
    
    # Calculate the pixel coordinates relative to the center of the image
    center_x = resolution_x / 2
    center_y = resolution_y / 2
    relative_x = pixel_x - center_x
    relative_y = pixel_y - center_y
    
    # Calculate the angle in radians
    angle_x = math.atan2(relative_x, resolution_x / (2 * math.tan(fov_radians / 2)))
    angle_y = math.atan2(relative_y, resolution_y / (2 * math.tan(fov_radians / 2)))
    
    # Convert the angle to degrees
    angle_x_degrees = math.degrees(angle_x)
    angle_y_degrees = math.degrees(angle_y)
    
    return angle_x_degrees, angle_y_degrees
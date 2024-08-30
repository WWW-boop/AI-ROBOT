# Example data points from the graph (approximate)
# Voltage (V) to Distance (cm)
voltage_to_distance_map = [
    (3.1, 2), (3.0, 4), (2.8, 6), (2.5, 8), (2.2, 10),
    (1.9, 12), (1.6, 14), (1.4, 16), (1.2, 18), (1.0, 20),
    (0.9, 22), (0.7, 24), (0.6, 26), (0.5, 28), (0.4, 30),
    (0.35, 32), (0.3, 34), (0.25, 36), (0.2, 38), (0.15, 40)
]

# Function to find the closest distance for a given voltage
def voltage_to_distance(voltage):
    closest_distance = None
    smallest_diff = float('inf')
    
    for v, d in voltage_to_distance_map:
        diff = abs(v - voltage)
        if diff < smallest_diff:
            smallest_diff = diff
            closest_distance = d
    
    return closest_distance

# Example usage
input_voltage = 2.0  # Replace with the actual voltage input
output_distance = voltage_to_distance(input_voltage)
print(f'Distance for voltage {input_voltage} V is approximately {output_distance} cm')

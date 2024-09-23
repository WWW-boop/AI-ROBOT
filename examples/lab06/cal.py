#60 120 180
#60cm w = 62 px h = 156 px
#120cm w = 31 px h = 73 px
#180cm w = 19 px h = 53 px

import matplotlib.pyplot as plt
import numpy as np

# Function definitions
def calculate_kxf_x(dpx_x, X, Z):
    return (dpx_x * Z) / X

def calculate_kxf_y(dpx_y, Y, Z):
    return (dpx_y * Z) / Y

def calculate_pixel_distance_x(k_x_f_x, X, Z):
    return (X * k_x_f_x) / Z

def calculate_pixel_distance_y(k_x_f_y, Y, Z):
    return (Y * k_x_f_y) / Z

# Given values for different distances
distances = [60, 120, 180]  # cm
dpx_x_values = [63, 33, 22]  # pixels
dpx_y_values = [167, 80, 53]  # pixels
X = 5.8  # cm (width of Coke can)
Y = 14.6  # cm (height of Coke can)

# Lists to store the calculated constants
k_x_f_x_values = []
k_x_f_y_values = []

# Calculate k_xf and k_yf for each distance
for Z, dpx_x, dpx_y in zip(distances, dpx_x_values, dpx_y_values):
    k_x_f_x = calculate_kxf_x(dpx_x, X, Z)
    k_x_f_y = calculate_kxf_y(dpx_y, Y, Z)
    k_x_f_x_values.append(k_x_f_x)
    k_x_f_y_values.append(k_x_f_y)
    print(f"Z = {Z} cm -> k_x_f_x = {k_x_f_x:.2f} pixel-cm, k_x_f_y = {k_x_f_y:.2f} pixel-cm")

# Calculate the averages
average_k_x_f_x = np.mean(k_x_f_x_values)
average_k_x_f_y = np.mean(k_x_f_y_values)

print(f"\nAverage k_x_f_x = {average_k_x_f_x:.2f} pixel-cm")
print(f"Average k_x_f_y = {average_k_x_f_y:.2f} pixel-cm")

# Test the formulas for new distances
new_distances = [75, 100, 125]

print("\nTesting new distances:")
predicted_pixel_distances_x = []
predicted_pixel_distances_y = []
for Z in new_distances:
    pixel_distance_x = calculate_pixel_distance_x(average_k_x_f_x, X, Z)
    pixel_distance_y = calculate_pixel_distance_y(average_k_x_f_y, Y, Z)
    predicted_pixel_distances_x.append(pixel_distance_x)
    predicted_pixel_distances_y.append(pixel_distance_y)
    print(f"At Z = {Z} cm -> Width in pixels = {pixel_distance_x:.2f}, Height in pixels = {pixel_distance_y:.2f}")

# Plotting
# Combine data for plotting
all_physical_distances = distances + new_distances
all_pixel_distances_x = dpx_x_values + predicted_pixel_distances_x
all_pixel_distances_y = dpx_y_values + predicted_pixel_distances_y

# Linear regression for both width and height
coefficients_x = np.polyfit(all_physical_distances, all_pixel_distances_x, 1)
linear_fit_x = np.polyval(coefficients_x, all_physical_distances)

coefficients_y = np.polyfit(all_physical_distances, all_pixel_distances_y, 1)
linear_fit_y = np.polyval(coefficients_y, all_physical_distances)

# Plotting
plt.figure(figsize=(10, 8))

# Plot for measured data only
plt.subplot(3, 1, 1)
plt.plot([60, 120, 180], [63, 33, 22], 'bo-', label='Width')
plt.plot([60, 120, 180], [167, 80, 53], 'go-', label='Height')
plt.title('Measured Width and Height for 60cm, 120cm, 180cm')
plt.xlabel('Distance (cm)')
plt.ylabel('Pixels')
plt.legend()
plt.grid(True)

# Width plot
plt.subplot(3, 1, 2)
plt.scatter(distances, dpx_x_values, color='blue', label='Measured (pixels)')  # Set measured data same color
plt.scatter(new_distances[0], predicted_pixel_distances_x[0], color='orange', label='Predicted Width (75 cm)')
plt.scatter(new_distances[1], predicted_pixel_distances_x[1], color='purple', label='Predicted Width (100 cm)')
plt.scatter(new_distances[2], predicted_pixel_distances_x[2], color='green', label='Predicted Width (125 cm)')
plt.plot(all_physical_distances, linear_fit_x, color='red', label='Linear Fit Width')
plt.title('Physical Distance vs. Width in Pixels')
plt.xlabel('Physical Distance (cm)')
plt.ylabel('Width in pixels')
plt.legend()
plt.grid()

# Height plot
plt.subplot(3, 1, 3)
plt.scatter(distances, dpx_y_values, color='blue', label='Measured (pixels)')  # Set measured data same color
plt.scatter(new_distances[0], predicted_pixel_distances_y[0], color='orange', label='Predicted Height (75 cm)')
plt.scatter(new_distances[1], predicted_pixel_distances_y[1], color='purple', label='Predicted Height (100 cm)')
plt.scatter(new_distances[2], predicted_pixel_distances_y[2], color='green', label='Predicted Height (125 cm)')
plt.plot(all_physical_distances, linear_fit_y, color='red', label='Linear Fit Height')
plt.title('Physical Distance vs. Height in Pixels')
plt.xlabel('Physical Distance (cm)')
plt.ylabel('Height in pixels')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
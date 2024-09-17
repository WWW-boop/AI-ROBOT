#60 120 180
#60cm w = 63 px h = 167 px
#120cm w = 33 px h = 80 px
#180cm w = 22 px h = 53 px
#real coke w = 5.8 cm , h = 14.6 cm



def calculate_kxf_x(dpx_x, X, Z):
    k_x_f_x_value = (dpx_x * Z) / X
    return k_x_f_x_value

def calculate_kxf_y(dpx_y, Y, Z):
    k_x_f_y_value = (dpx_y * Z) / Y
    return k_x_f_y_value

def calculate_pixel_distance_x(k_x_f_x, X, Z):
    pixel_distance_x = (X * k_x_f_x) / Z
    return pixel_distance_x

def calculate_pixel_distance_y(k_x_f_y, Y, Z):
    pixel_distance_y = (Y * k_x_f_y) / Z
    return pixel_distance_y

# Given values for different distances
distances = [60, 120, 180]
dpx_x_values = [63, 33, 22]
dpx_y_values = [167, 80, 53]
X = 5.8  # cm (width)
Y = 14.6  # cm (height)

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
average_k_x_f_x = sum(k_x_f_x_values) / len(k_x_f_x_values)
average_k_x_f_y = sum(k_x_f_y_values) / len(k_x_f_y_values)

print(f"\nAverage k_x_f_x = {average_k_x_f_x:.2f} pixel-cm")
print(f"Average k_x_f_y = {average_k_x_f_y:.2f} pixel-cm")

# Test the formulas for new distances
new_distances = [75, 100, 125]

print("\nTesting new distances:")
for Z in new_distances:
    pixel_distance_x = calculate_pixel_distance_x(average_k_x_f_x, X, Z)
    pixel_distance_y = calculate_pixel_distance_y(average_k_x_f_y, Y, Z)
    print(f"At Z = {Z} cm -> Width in pixels = {pixel_distance_x:.2f}, Height in pixels = {pixel_distance_y:.2f}")

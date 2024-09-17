#60 120 180
#60cm w = 63 px h = 167 px
#120cm w = 33 px h = 80 px
#180cm w = 22 px h = 53 px

def calculate_kxf_x(dpx_x, X, Z):

    k_x_f_x_value = (dpx_x * Z) / X
    return k_x_f_x_value

def calculate_kxf_y(dpx_y, Y, Z):

    k_x_f_y_value = (dpx_y * Z) / Y
    return k_x_f_y_value

dpx_x = 63
dpx_y = 167  
X = 5.8
Y = 14.6     
Z = 60     

k_x_f_x_value = calculate_kxf_x(dpx_x, X, Z)
k_x_f_y_value = calculate_kxf_y(dpx_y,Y,Z)
print(f"k_x_f_x = {k_x_f_x_value:.2f} pixel-cm")
print(f"k_x_f_y = {k_x_f_y_value:.2f} pixel-cm")
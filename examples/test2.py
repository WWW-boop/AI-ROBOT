def convert_to_V(ssR,ssL):
    # Assuming the sensor value (ssR) needs to be converted to centimeters.
    ad_data_vo_ssr = (ssR * 3) / 1023
    ad_data_vo_ssl = (ssL * 3) / 1023
    return ad_data_vo_ssr , ad_data_vo_ssl

def convert_to_cm(voltage):
    if voltage > 1.4:
        cm = (voltage - 4.2) / -0.31
    elif 1.4 >= voltage >= 0.6:
        cm = (voltage - 2.03) / -0.07
    else:
        cm = (voltage - 0.95) / -0.016
    
    return cm


# Example usage:
ssR = 200  # Example sensor reading
ssL = 200 

dis_cm = convert_to_cm(vaR)
dis_cm1 = convert_to_cm(vaL)
print(f"Distance: {dis_cm}  cm")
print(f"Distance: {dis_cm1}  cm")



# global adc_1, status_ss_1
#     adc_1 = ep_sensor_adaptor.get_adc(id=1, port=2)
#     adc_2 = (adc_1 * 3) / 1023  # process to cm unit

#     if adc_2 > 1.4:
#         adc_cm = (adc_2 - 4.2) / -0.31
#     elif 1.4 >= adc_2 >= 0.6:
#         adc_cm = (adc_2 - 2.03) / -0.07
#     else:
#         adc_cm = (adc_2 - 0.95) / -0.016
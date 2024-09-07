import cv2
import robomaster
from robomaster import robot, blaster, camera
import time
import numpy as np

def calculate_cosine_similarity(tmpl, wnd):
    dot_prod = np.dot(tmpl, wnd)
    norm_tmpl = np.linalg.norm(tmpl)
    norm_wnd = np.linalg.norm(wnd)
    if norm_tmpl == 0 or norm_wnd == 0:
        return 0
    return dot_prod / (norm_tmpl * norm_wnd)

def angle_data_callback(data):
    global angle_data
    angle_data = data

def convert_image_to_hsv(img):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv_img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    return hsv_img

def apply_hue_threshold(hsv_img):
    low_red_1 = np.array([0, 100, 100])
    up_red_1 = np.array([10, 255, 255])
    low_red_2 = np.array([170, 100, 100])
    up_red_2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv_img, low_red_1, up_red_1)
    mask2 = cv2.inRange(hsv_img, low_red_2, up_red_2)
    final_mask = cv2.bitwise_or(mask1, mask2)
    return final_mask

def extract_largest_contour(mask_img):
    cnts, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_cnt = max(cnts, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_cnt)
    cropped = mask_img[y:y+h, x:x+w]
    return cropped, [(x, y)], [(w, h)]

def load_templates():
    template_files = [
        r"RoboMaster-SDK\examples\pic\1.png",
        r"RoboMaster-SDK\examples\pic\2.png",
        r"RoboMaster-SDK\examples\pic\3.png",
        r"RoboMaster-SDK\examples\pic\4.png"
    ]

    templates = []
    for file in template_files:
        img = cv2.imread(file)
        hsv_img = convert_image_to_hsv(img)
        thresh_img = apply_hue_threshold(hsv_img)
        cropped_img, _, _ = extract_largest_contour(thresh_img)
        templates.append(cropped_img)

    return templates

def perform_sliding_window(img, templates, step=100, pad=10, scale_factors=[0.75, 1, 1.25, 1.5]):
    max_sim = 0
    optimal_position = (0, 0)
    optimal_size = (0, 0)

    img_padded = np.pad(img, ((pad, pad), (pad, pad), (0, 0)), mode="constant")
    hsv_img = convert_image_to_hsv(img_padded)
    thresh_img = apply_hue_threshold(hsv_img)

    for scale in scale_factors:
        for template in templates:
            tmpl_resized = cv2.resize(template, None, fx=scale, fy=scale)
            tmpl_h, tmpl_w = tmpl_resized.shape[:2]

            for y in range(0, thresh_img.shape[0] - tmpl_h + 1, step):
                for x in range(0, thresh_img.shape[1] - tmpl_w + 1, step):
                    wnd = thresh_img[y:y+tmpl_h, x:x+tmpl_w]
                    sim = calculate_cosine_similarity(tmpl_resized, wnd)

                    if sim > max_sim:
                        max_sim = sim
                        optimal_position = (x - pad - 10, y - pad - 10)
                        optimal_size = (tmpl_w, tmpl_h)

    return optimal_position, optimal_size, max_sim

if __name__ == "__main__":
    bot = robot.Robot()
    bot.initialize(conn_type="ap")
    cam = bot.camera
    gimbal = bot.gimbal
    blaster_unit = bot.blaster

    mid_x = 1280 / 2
    mid_y = 720 / 2

    cam.start_video_stream(display=False, resolution=camera.STREAM_720P)
    gimbal.sub_angle(freq=20, callback=angle_data_callback)

    # PID control constants
    kp = 0.353
    kd = 0.043
    ki = 0.723

    err_x_prev = 0.0
    err_y_prev = 0.0
    prev_time = time.time()

    integral_err_x = 0
    integral_err_y = 0

    recorded_data = []

    loop_count = 0

    templates = load_templates()
    gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    detect_start = None

    while True:
        frame = cam.read_cv2_image(strategy="newest", timeout=0.5)
        loc, size, similarity = perform_sliding_window(frame, templates)

        curr_time = time.time()

        if similarity > 0.9:
            x_pos, y_pos = loc
            width, height = size

            err_x = mid_x - (x_pos + width / 2)
            err_y = mid_y - (y_pos + height / 2)

            integral_err_x += err_x
            integral_err_y += err_y

            if loop_count >= 1:
                yaw_speed = kp * err_x
                pitch_speed = kp * err_y

                gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=-yaw_speed)

                recorded_data.append(
                    [curr_time] + list(angle_data) + [err_x, err_y, round(yaw_speed, 3), round(pitch_speed, 3)] + [x_pos, y_pos]
                )

            loop_count += 1
            err_x_prev = err_x
            err_y_prev = err_y
            prev_time = curr_time

            if detect_start is None:
                detect_start = curr_time
            elif curr_time - detect_start > 2:
                detect_start = None

            new_w = int(width * 0.7)
            new_h = int(height * 0.3)

            new_x = x_pos - new_w // 2
            new_y = int(y_pos - new_h // 1.5)
            new_w = width + new_w
            new_h = height + new_h

            cv2.rectangle(frame, (new_x, new_y), (new_x + new_w, new_y + new_h), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"Detected Object ({similarity:.2f})",
                (x_pos, y_pos - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )
        else:
            gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            detect_start = None

        cv2.imshow("Object Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.001)

    cv2.destroyAllWindows()
    cam.stop_video_stream()
    bot.close()

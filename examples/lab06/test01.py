#B  W = 5.8cm H = 14.6cm

#C = 
#D = 
#E = 
#F = 
#G = 
#H = 

import cv2


image = cv2.imread('RoboMaster-SDK\examples\pic\coca_60cm.jpg')


def draw_rectangle(event, x, y, flags, param):
    global x_start, y_start, drawing
    
    if event == cv2.EVENT_LBUTTONDOWN:
        x_start, y_start = x, y
        drawing = True
    
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            img_copy = image.copy()
            cv2.rectangle(img_copy, (x_start, y_start), (x, y), (0, 255, 0), 2)
            cv2.imshow('image', img_copy)
    
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        width = abs(x - x_start)
        height = abs(y - y_start)
        print(f'Width: {width} pixels, Height: {height} pixels')
        cv2.rectangle(image, (x_start, y_start), (x, y), (0, 255, 0), 2)
        cv2.imshow('image', image)


x_start, y_start = 0, 0
drawing = False


cv2.imshow('image', image)
cv2.setMouseCallback('image', draw_rectangle)

cv2.waitKey(0)
cv2.destroyAllWindows()

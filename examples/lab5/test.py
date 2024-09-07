import cv2

templates_ = [
        r"RoboMaster-SDK\examples\pic\coke-1block.jpg",
        r"RoboMaster-SDK\examples\pic\coke-100cm.jpg",
        r"RoboMaster-SDK\examples\pic\coke-3block.jpg",
        r"RoboMaster-SDK\examples\pic\coke-4block.jpg"
    ]
template_list = [cv2.imread(temp) for temp in templates_]  # โหลดรูปภาพ
print(template_list)
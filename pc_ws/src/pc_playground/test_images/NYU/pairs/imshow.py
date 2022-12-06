import cv2 as cv
if __name__ == "__main__":
    depth_path = "./depth_1.pgm"
    depth_img = cv.imread(depth_path)
    cv.imshow("depth", depth_img)

    rgb_path = "./rgb_1.ppm"
    rgb_img = cv.imread(rgb_path)
    cv.imshow("rgb", rgb_img)
    cv.waitKey(0)

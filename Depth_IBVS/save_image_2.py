import os
import cv2 as cv

def save_image(img, folder_path, file_name):
    # Create folder if it doesn't exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Save image
    file_path = os.path.join(folder_path, file_name)
    cv.imwrite(file_path, img)

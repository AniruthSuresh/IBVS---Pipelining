import cv2
import numpy as np

# Function to capture images at different focus distances
def capture_images(camera, focus_distances):
    images = []
    for distance in focus_distances:
        # Set the focus distance
        camera.set(cv2.CAP_PROP_FOCUS, distance)
        # Capture an image
        ret, frame = camera.read()
        if ret:
            images.append(frame)
        else:
            print("Error capturing image at focus distance:", distance)
    return images

# Function to estimate depth from focus
def depth_from_focus(images):
    # Convert images to grayscale
    gray_images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images]

    # Calculate image sharpness (variance of Laplacian)
    sharpness_scores = [cv2.Laplacian(gray_image, cv2.CV_64F).var() for gray_image in gray_images]

    # Normalize sharpness scores
    normalized_scores = np.array(sharpness_scores) / max(sharpness_scores)

    # Estimate depth based on sharpness scores (higher score indicates closer focus distance)
    depth = normalized_scores.argmax()

    return depth

# Main function
def main():
    # Open camera
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Unable to open camera.")
        return

    # Define focus distances (in camera-specific units)
    focus_distances = [100, 150, 200]  # Example focus distances

    # Capture images at different focus distances
    images = capture_images(camera, focus_distances)

    # Estimate depth from focus
    estimated_depth = depth_from_focus(images)
    print("Estimated depth:", estimated_depth)

    # Release camera
    camera.release()

if __name__ == "__main__":
    main()



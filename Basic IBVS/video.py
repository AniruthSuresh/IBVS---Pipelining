import os
import subprocess

def images_to_video(image_folder, video_name, fps=30):
    # Check if ffmpeg is installed
    ffmpeg_installed = subprocess.run(['ffmpeg', '-version'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if not ffmpeg_installed:
        print("Error: ffmpeg is not installed. Please install ffmpeg to convert images to video.")
        return
    
    # Generate the command to convert images to video using ffmpeg
    command = ['ffmpeg', '-r', str(fps), '-pattern_type', 'glob', '-i', f'{image_folder}/*.png', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', video_name]
    
    # Execute the command
    subprocess.run(command)

# Example usage:
# Assuming your images are stored in the 'img' folder and you want to create a video named 'output.mp4'
# images_to_video('img', 'output.mp4')

def make_video_from_images(image_folder, video_name, fps=30):
    # Get a list of all image files in the folder
    image_files = sorted([f for f in os.listdir(image_folder) if f.endswith('.png')])

    # Call the conversion function to create the video
    images_to_video(image_folder, video_name, fps)

# Example usage:
make_video_from_images('Images', 'output.mp4')

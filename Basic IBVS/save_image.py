import pybullet as p
from PIL import Image

def save_image(pb: p, index: int) -> None:
    img = pb.getCameraImage(1024, 1024, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
    rgbBuffer = img[2]
    rgbim = Image.fromarray(rgbBuffer)
    rgbim.save(f'/home/aniruth/Desktop/RRC/VisualServoing_Depth/IBVS-pybullet/visual-servoing/IBVS/Images/Imrgbimage_{index}.png')



from PIL import Image
import numpy
import numpy as np
import cv2



def converting_image_to_array():
    image = Image.open("costmap.png")
    array = numpy.array(image)

print(array) # что же всплыло ? 

def main():
    converting_image_to_array()


if __name__ == '__main__':
    main()


    ## да-да это вооще не то что нужно , но я начал поПЫТКИ

    
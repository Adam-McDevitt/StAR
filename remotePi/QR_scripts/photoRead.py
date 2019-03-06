import os
import pyzbar.pyzbar as pz
import numpy as np
import cv2
import sys
import subprocess
import time

deleteCommand = "rm image.jpg"

def decodepz(im) :
    # Finds QR codes
    decodedObjects = pz.decode(im)
    # Print results
    for obj in decodedObjects:
        print(obj.data)
    # print("About to return data")
    return decodedObjects

os.system("fswebcam -r 1280x720 --no-banner image.jpeg")


# print(os.path.getsize("image.jpeg"))

if os.path.isfile("image.jpeg"):
    # read file
    print("File exists")


    im = cv2.imread("image.jpeg")
    decodedObjects = decodepz(im)
else:
    print("No file Exixts")

# while not os.path.exists("image.jpeg"):
    # time.sleep(1)


# os.system(deleteCommand)

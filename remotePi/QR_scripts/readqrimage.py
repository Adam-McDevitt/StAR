from __future__ import print_function
import pyzbar.pyzbar as pz
import numpy as np
import cv2
#from matplotlib import pyplot as plt
import sys

def decodepz(im) :
    # Finds QR codes
    decodedObjects = pz.decode(im)
    # Print results
    for obj in decodedObjects:
        print(obj.data)
    return decodedObjects

input = str(sys.argv[1])
im = cv2.imread(input)
decodedObjects = decodepz(im)

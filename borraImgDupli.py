import cv2
import glob
from os import system
import sys
import numpy as np

directory = sys.argv[1]

img_hashes = set()

for path in glob.glob(directory + "*.jpg"):
    img = cv2.imread(path)
    img_hash = hash(str(img))
    if img_hash in img_hashes:
        system("rm " + path)
    else:
        img_hashes.add(img_hash)
#!/usr/bin/python

import cv2
import numpy as np
from matplotlib import pyplot as plt



def main():
    # lade Beispielbild
    gray_img = cv2.imread('hand_segmented.png', 0)                  # Ich gebe dem Bild direkt den Datentyp Grauwertbild mit der 0. Ansonsten waere das ein graues BGR Bild

    kernel = np.ones((10, 10), dtype=np.uint8)*255                  # Ich definiere hier einen 10x10 Kernel mit allen Eintraegen 255
    result_img = cv2.morphologyEx(gray_img, cv2.MORPH_OPEN, kernel) # Wende den morphologisches Oeffnen - Algorithmus an

    # Zeige Input und Ergebnis im Vergleich an
    fig, ax = plt.subplots(1, 2)
    ax[0].imshow(gray_img, cmap='gray')
    ax[0].set_title('input image')

    ax[1].imshow(result_img, cmap='gray')
    ax[1].set_title('result after morphologic opening')

    plt.show()

if __name__ == "__main__" :
    main()
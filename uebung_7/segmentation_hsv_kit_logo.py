#!/usr/bin/python

import numpy as np
from matplotlib import pyplot as plt

import cv2



def main():

    bgr_img = cv2.imread('armar_4.png')
    rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

    plt.imshow(rgb_img)
    plt.show()

    hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([80, 150, 150])
    upper_green = np.array([90, 255, 255])

    mask = cv2.inRange(hsv_img, lower_green, upper_green)
    plt.imshow(mask, cmap='gray')
    plt.show()

    segment = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)
    result = cv2.cvtColor(segment, cv2.COLOR_HSV2RGB)

    plt.imshow(result)
    plt.show()





if __name__ == "__main__":
    main()
    # rufe main() auf, wenn das Modul (das Python Skript) als Hauptprogramm aufgerufen wird
    #
    # Erklaerung:
    # Jedes Modul hat eine Systemvariable namens __name__ . Diese wird zur Laufzeit mit dem
    # Namen des Namespace beschrieben, in dem das Modul ausgefuehrt wird.
    # Wird das Modul allein aufgerufen (nicht importiert), so wird das Modul im Namespace __main__ aufgerufen
    # also quasi als Hauptprogramm verwendet.
    # Die Methode wird also nur dann ausgefuehrt, wenn das Modul nicht importiert wurde.

    # Allgemein wichtig:
    # Der ausfuehrbare Code (also das, was im Skript ausserhalb von Funktionen, Klassen, etc. steht)
    # wird ausgefuehrt, sobald das Modul importiert wird, oder als Modul allein aufgerufen wird.

#!/usr/bin/python

import cv2
from matplotlib import pyplot as plt

def main():
    # Lade Beispielbild als Grauwertbild mittels der imread-Option 0
    gray_img = cv2.imread('armar_4.png', 0)
    result_canny = cv2.Canny(gray_img, 100, 200)

    # Definiere Subplots und schreibe Plot-Optionen mit deren Handels (ax[i])
    fig, ax = plt.subplots(1, 2)
    ax[0].imshow(gray_img, cmap='gray')
    ax[0].set_title('input image')

    ax[1].imshow(result_canny, cmap='gray')
    ax[1].set_title('canny edge detection')

    # Zeige die Bilder an
    plt.show()



if __name__ == "__main__":
    main()

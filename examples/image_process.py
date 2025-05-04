import cv2
import os



if __name__ == "__main__":

    dir = "Fast-ACVNet"
    image_name = "BotanicGarden-right.png"
    width = 960
    height = 512
    img = cv2.imread(os.path.join(dir, image_name))

    # Resize to 800x600
    resized = cv2.resize(img, (width, height))

    # Save resized image
    cv2.imwrite(os.path.join(dir, f'BotanicGarden-right-{width}x{height}.png'), resized)


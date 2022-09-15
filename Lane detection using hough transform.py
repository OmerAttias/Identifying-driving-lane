import cv2
import numpy as np



def region_of_interest(image, triangle):
    # return an image that equal
    # dimennsioned to the original image
    # and complite zeroed
    mask = np.zeros_like(image)
    # def the color of the polygon lines
    match_mask_color = 255
    # function that take our triangle shape,
    # from the given vertices and Poses it on
    # the zeroed masked
    # all the lines will be colored to white
    cv2.fillPoly(mask, triangle, match_mask_color)

    # poses the final mask on the origin picture
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def linesImageDrawing(image, lines):
    # keeps the algoritem runing
    # even if a line isnt recognize
    if lines is not None:
        for line in lines:
            # finding 2 extreame points on the line
            for x1, y1, x2, y2 in line:
                # print the line on the screen
                cv2.line(image, (x1,y1), (x2,y2), (0, 255, 0), thickness=5)
                # fine line middle x point
                line_middle_x = int((x1 + x2) / 2)
                #print(xC,yC)
                # term for turn
                # if on of the lines is near the middle car point
                # it means the car is in turn position
                if  335 < line_middle_x < 345:
                    print('Turn')
    return image


def image_process(image):
    #moving the image to color model 'gray scale'
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # put an binary treshhold that keeps Shades of gray
    # that are close to white and removing the rest.
    canny_image = cv2.Canny(gray_image, 140, 200)
    # define a 3 vertices for
    triangle_vertices = [
        (140, 360),
        (340, 255),
        (590, 360)
    ]
    # put a circle that define the car center
    cv2.circle(image, (340, 300), 2, (0, 0, 255), 5)
    # sending the image to a function which keeps
    # a triangle shape that the road inside of
    # and throw the rest of the image
    cropped_image = region_of_interest(canny_image,
                    np.array([triangle_vertices], np.int32),)
    # after keeping the triangle,
    # function that monitoring lines with
    # hough transform technique
    lines = cv2.HoughLinesP(cropped_image,
                            # the distance from image start point
                            rho=1,
                            # the angle from image start point
                            theta=np.pi/180,
                            # threshold on line votes,
                            # which is rho and theta vector
                            threshold=20,
                            lines=np.array([]),
                            minLineLength=25,
                            # line gap for making
                            # a Continuously line
                            # if needed
                            maxLineGap=500)
    # sending to a function which draw the founed lines
    # on the frame
    image_with_lines = linesImageDrawing(image, lines)
    return image_with_lines


# get a video capture object for the camera
cap = cv2.VideoCapture('test5.mp4')

#Returns true if video capturing has been initialized already.
while cap.isOpened():
    #reads the data per frame. 'ret' refer to a any kind of boolean return and 'img' refer the the each img that returns
    ret, frame = cap.read()
    frame = image_process(frame)
    # ploting the video
    cv2.imshow('frame', frame)

    #display a window for given milliseconds or until any key is pressed
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
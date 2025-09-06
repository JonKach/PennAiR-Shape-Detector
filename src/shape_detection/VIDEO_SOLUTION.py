import cv2
import numpy as np
 
cap = cv2.VideoCapture("detect_resources/dynamic_video.mp4")

while True: 
    ret, frame = cap.read()
    img = frame
    img = cv2.resize(img, (1532, 796)) #Resize images to a standard size so that text looks good
    copy = img.copy() #Keep the original image to write graphics onto later
    copy = cv2.cvtColor(copy, cv2.COLOR_RGB2GRAY, None) # Load image in grayscale

    #Snippet to auto-tune Canny Edge Detector
    med = np.median(copy)
    sigma = 0.33
    lower = int(max(0, (1.0-sigma)*med))
    upper = int(min(255, (1.0+sigma)*med))

    edges = cv2.Canny(copy, lower, upper) #Detect object edges

    '''Run a second edge detection strategy. This is mostly redundant but I've found it can lead to a more stable
    image by thickening edge boundaries (although it is not required for accurate detections). I've included it because 
    the Sobel operator is computationally inexpensive.'''
    sobelx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)  
    sobely = cv2.Sobel(edges, cv2.CV_64F, 0, 1, ksize=3)  
    gradient_magnitude = cv2.magnitude(sobelx, sobely)
    
    # Convert to uint8
    gradient_magnitude = cv2.convertScaleAbs(gradient_magnitude)

    #Heavily blur the image so small edges join together, leaving only shapes as the major elements
    blur = cv2.GaussianBlur(gradient_magnitude, (17,17), 0)

    '''Slightly dilate the image to cover up small black gaps in the background of the blurred image
    (which, if left in, could lead to a "jumpy" look for the detected outlines)'''
    dilate = cv2.dilate(blur, (3,3 ), iterations=1)

    #Threshold to isolate the shapes from the background and convert the image to a binary image.
    retval, threshed = cv2.threshold(dilate, 10, 255, cv2.THRESH_BINARY_INV)

    #Find the most external outlines of each shape
    contours, hierarchy = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Filter contours by area (only keep the largest shapes)
    selected_contours = []
    for contour in contours:
        if(cv2.contourArea(contour) > 2000):
            selected_contours.append(contour)

    #Now that we've found the individual contours, it time to merge shapes when they meet

    #Prepare a black image for the merging process
    black_img = np.zeros_like(img)
    black_img = cv2.cvtColor(black_img, cv2.COLOR_BGR2GRAY)

    #Draw contours onto black image
    cv2.drawContours(black_img, selected_contours, -1, (255, 255, 255), cv2.FILLED)

    #Heavily blur to eliminate "ghost borders" between shapes (that were created by contour boundaries)
    blur_2 = cv2.GaussianBlur(black_img, (25, 25), 40)

    #Convert the blurred image back to a binary image
    ret, binary = cv2.threshold(blur_2, 40, 255, cv2.THRESH_BINARY)

    #Find joined countours
    joined_contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Draw joined contours onto the original color image
    cv2.drawContours(img, joined_contours, -1, (57.12,255,19.89), 2)

    #Define variables required for depth calculation
    camera_matrix = [[2564.3186869,0,0],
                 [0,2569.70273111,0],
                 [0, 0, 1]]
    CIRCLE_RADIUS_IN_PX = 92
    CIRCLE_RADIUS_CM = 10

    for contour in joined_contours:
        moments = cv2.moments(contour) #returns a dictionary of moments
        if moments['m00'] != 0:
            Cx = int(moments['m10'] / moments['m00']) #find center
            Cy = int(moments['m01'] / moments['m00'])
            cv2.circle(img, (Cx, Cy), 4, (255, 255, 255), -1)
            cv2.putText(img, "Center", (Cx-20, Cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            #calculate depth using standard formulas (D = F x W / P) and circle as reference
            depth = camera_matrix[0][0] * CIRCLE_RADIUS_CM / CIRCLE_RADIUS_IN_PX
            x = (Cx - camera_matrix[0][2]) * depth / camera_matrix[0][0] #convert x and y to real world coords
            y = (Cy - camera_matrix[1][2]) * depth / camera_matrix[1][1]

            cv2.putText(img, f"Coords: [{int(x)}, {int(y)}, {int(depth)}]", (Cx-20, Cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)


    # Display result
    img = cv2.resize(img, None, fx=0.5, fy=0.5)
    cv2.imshow("Result", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
cap.release()
cv2.destroyAllWindows()
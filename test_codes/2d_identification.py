import numpy as np
import cv2 as cv

'''Image input'''
img = cv.imread('test_pic.jpg')
#img = cv.imread('diff_shapes.png')
#img = cv.imread('diff_shapes2.png')
final = img

'''Image properties'''
height_img , width_img , channel_img = img.shape
print(height_img , width_img , channel_img)


'''masking and other functions'''
grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
_, thresh = cv.threshold(grey, 185, 250, cv.THRESH_BINARY)
mask = cv.erode(thresh , kernel=np.ones((3,3),np.uint8)) #got clean img here

'''Contour variables'''
font  = cv.FONT_HERSHEY_COMPLEX
font_thickness = 2

'''Contour identification'''
contours , heirarchy = cv.findContours(mask, cv.RETR_TREE , cv.CHAIN_APPROX_NONE)
for cnts in contours:
    approx = cv.approxPolyDP(cnts , 0.01*cv.arcLength(cnts , False) , True)  #true for To close the arc , countour closed function 
    cv.drawContours(grey, [approx], 0 , (0) , 2)

    x = approx.ravel()[0] ; y = approx.ravel()[1]
    x = height_img//2 ; y = width_img//2
    xc, yc , w, h = cv.boundingRect(approx)
    cnt_area = cv.contourArea(cnts)

    if cnt_area > 500:
        if len(approx) == 3:
            cv.putText(final ,'Triangle', (x,y),font, fontScale=1 ,color=(0),thickness=font_thickness)

        elif len(approx) == 4:
            aspect_ratio = float(w) / h
            if 0.95 <= aspect_ratio <= 1.5:
                cv.putText(final ,'Square', (x,y),font, fontScale=1 ,color=(0),thickness=font_thickness)
            else:
                cv.putText(final ,'Rectangle', (x,y),font, fontScale=1 ,color=(0),thickness=font_thickness)

        elif len(approx) == 5:
            cv.putText(final ,'Pentagon', (x,y),font, fontScale=1 ,color=(0),thickness=font_thickness)
        
        elif len(approx) == 6:
            cv.putText(final ,'Hexagon', (x,y),font, fontScale=1 ,color=(0),thickness=font_thickness)

        else:
            cv.putText(final ,'Circle', (x,y),font, fontScale=1 ,color=(0),thickness=font_thickness)


print(f'Width:{w} ,Height:{h}, area:{cnt_area}')

'''Show functions'''
cv.imshow("grey", grey)
cv.imshow("Threshold", thresh)
cv.imshow("mask", mask)
cv.imshow("final window", final)

'''wait key'''
cv.waitKey(0)
cv.destroyAllWindows()

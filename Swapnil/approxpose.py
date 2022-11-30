#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import glob

def showimg(img,name="image"):
    cv.imshow(name,img)
    cv.waitKey(0)
    cv.destroyAllWindows()
    cv.waitKey(1)

def pltshow(img,title="image"):    
    plt.imshow(img)
    plt.axis('off')
    plt.title(title)
    plt.show()

dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250) 
parameters = cv.aruco.DetectorParameters_create()



# markerImage = np.zeros((200,200), dtype=np.uint8)
# markerImage = cv.aruco.drawMarker(dictionary,3,200,markerImage,1)
# #cv.imwrite("marker33.png",markerImage)

# showimg(markerImage)

# a = "marker" + str(n)

# cv.imwrite("marker3.png",markerImage)

# cap = cv.VideoCapture(0)

# if not cap.isOpened():
#     print("camera closed")
#     exit()
    
# while True:
#     #frame capturing
#     ret, frame = cap.read()
    
#     #ret=true when frame read is correct
#     if not ret:
#         print("Can't receive frame (stream end?). Exiting ...")
#         break
        
#     #now operating on the frame image
#     #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#     #markerCorners = []
#     #markerIds = []
#     markerCorners, markerIds , rejectedCandidates = cv.aruco.detectMarkers(frame,dictionary,parameters=parameters)
#     if markerIds is not None:
#         print(np.shape(markerCorners))
#     #if len(markerIds) > 0:
#     cv.aruco.drawDetectedMarkers(frame,markerCorners,markerIds)
    
#     cv.imshow('frame',frame)
        
#     if cv.waitKey(1) == ord('q'):
#         break
    

   
    
# #release the camera
# cap.release()
# cv.destroyAllWindows()

# markerCorners, markerIds , rejectedCandidates = cv.aruco.detectMarkers(frame,dictionary,parameters=parameters)

# showimg(frame)

# markerCorners

# ls

# al = np.zeros((1,2,3),np.float32)

# np.shape(al)

# Camera Calibration 

#import numpy as np
#import cv2 as cv

# termination criteria
# criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objp = np.zeros((6*9,3), np.float32)
# objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# # Arrays to store object points and image points from all the images.
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.
# images = glob.glob('*.jpeg')
# for fname in images:
#     img = cv.imread(fname)
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv.findChessboardCorners(gray, (9,6), None)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)
#         corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
#         imgpoints.append(corners2)
#         # Draw and display the corners
#         cv.drawChessboardCorners(img, (9,6), corners2, ret)
#         cv.imshow('img', img)
#         cv.waitKey(500)
# cv.destroyAllWindows()

# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# print('camera matrix = ')
# print(mtx)
# print(" ")
# print('distortion vector = ')
# print(dist)


# In[1]:


# dist


# In[ ]:





# In[9]:


# len(objpoints)


# In[ ]:





# In[8]:


# np.shape(objpoints)


# In[9]:


# np.shape(rvecs)


# In[10]:


# np.shape(tvecs)


# In[7]:


# len(rvecs)


# In[17]:


# cv::VideoCapture inputVideo;
# inputVideo.open(0);
# cv::Mat cameraMatrix, distCoeffs;
# // You can read camera parameters from tutorial_camera_params.yml
# readCameraParameters(filename, cameraMatrix, distCoeffs); // This function is located in detect_markers.cpp
# cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
# while (inputVideo.grab()) {
#     cv::Mat image, imageCopy;
#     inputVideo.retrieve(image);
#     image.copyTo(imageCopy);
#     std::vector<int> ids;
#     std::vector<std::vector<cv::Point2f>> corners;
#     cv::aruco::detectMarkers(image, dictionary, corners, ids);
#     // if at least one marker detected
#     if (ids.size() > 0) {
#         cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
#         std::vector<cv::Vec3d> rvecs, tvecs;
#         cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
#         // draw axis for each marker
#         for(int i=0; i<ids.size(); i++)
#             cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
#     }
#     cv::imshow("out", imageCopy);
#     char key = (char) cv::waitKey(waitTime);
#     if (key == 27)
#         break;
# }


# In[ ]:


# rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
#                                                                        distortion_coefficients)
#             # Draw a square around the markers
#             cv2.aruco.drawDetectedMarkers(frame, corners) 

#             # Draw Axis
#             cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  


# In[9]:


cap = cv.VideoCapture(0)

if not cap.isOpened():
    print("camera closed")
    exit()
#GET CAMERA AND DISTORTION MATRICES    
while True:
    #frame capturing
    ret, frame = cap.read()
    qq
    #ret=true when frame read is correct
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
        
    #now operating on the frame image
    #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    markerCorners, markerIds , rejectedCandidates = cv.aruco.detectMarkers(frame,dictionary,parameters=parameters)
    if markerIds is not None:
        #print(np.shape(markerCorners))
    #if len(markerIds) > 0:
        cv.aruco.drawDetectedMarkers(frame,markerCorners,markerIds)
        number = np.shape(markerCorners)[0]
        #ROTVECS = np.zeros([number,3,1], np.float32)
        #TRANVECS = np.zeros([number,3,1],np.float32)
        #OBJPTS = np.zeros([number,3,1],np.float32)#[0.0]*4*len(markerIds)
        
        ROTVECS,TRANSVECS,OBJPTS = cv.aruco.estimatePoseSingleMarkers(markerCorners,0.05,mtx,dist)
        #print(ROTVECS)
        for i in range(0,len(markerIds)):
            cv.drawFrameAxes(frame,mtx,dist,ROTVECS[i],TRANSVECS[i],0.01)

    cv.imshow('frame',frame)

    if cv.waitKey(1) == ord('q'):
            break
    

   
    
#release the camera
cap.release()
cv.destroyAllWindows()


# In[ ]:





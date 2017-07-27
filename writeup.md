**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[chessboard]: ./output_images/chessboard.png "Chessboard"
[original]: ./test_images/test2.jpg "Sample Image"
[undistorted]: ./output_images/undistorted.png "Undistorted"
[binary_thresholded]: ./output_images/binary_thresholded.png "Binary Thresholded"
[warped]: ./output_images/warped.png "Warped"
[warped_binary]: ./output_images/warped_binary.png "Warped Binary"
[fitted]: ./output_images/fitted.png "Fitted"
[result]: ./output_images/result.png "Result"
[video1]: ./project_video_result.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the second code cell of the IPython notebook located in "./adv_lane_finding.ipynb".  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection by `cv2.findChessboardCorners()`.

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![chessboard][chessboard]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

I will use this example image below for the all following pipeline steps:

![original][original]

After undistortion:

![undistorted][undistorted]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps in cell 5 and 6 of `adv_lane_finding.ipynb`). Specifically speaking, I use the x-axis gradient, the magnitude and direction of gradient vector, and the HLS color space thresholding. Here's this step's result for the sample image:

![binary_thresholded][binary_thresholded]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The corresponding code is the cell 7,8,9 of the IPython notebook. The `perspective_trans()` function accepts an image, and returns the transformed image and the tranformation matrix and inverse matrix.

Choosing the source area is a little tricky. I tried the straight lines image (like `./test_images/straight_lines1.jpg`) multiple times, to make the resulted straight lanes vertial and parallel.

Here are the hard-coded coordinates:

```python
    far_x_offset = 95
    near_y_offset = 30
    near_x_offset = 4
    src = [[640 - far_x_offset, 460],
           [640 + far_x_offset, 460],
           [1280 - near_x_offset, 720 - near_y_offset],
           [near_x_offset, 720 - near_y_offset]]
    dst = [[0,0], [1280,0],[1280,720],[0,720]]
```

Then I verified the transform on the same example image as above, to make sure the lanes appear parallel in the warped image.

Here is the warped color image:

![warped][warped]

Here is the warped binary-thresholded image:

![warped_binary][warped_binary]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

I mostly copied the code from the course material - basically find the peaks of left and right halves of the histogram as starting points of lane lines, then use 9 sliding windows to get all pixels of lanes, then fit the pixels with 2nd order polynomials. The code is at the cell 11 of IPython notebook. 

Here is the result (pixels of two lanes are colored as red and blue, and the yellow curve lines are the fitted polynomials): 

![fitted][fitted]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did it at the bottom part of cell 11 of IPython notebook:

```python
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension
    y_eval = np.max(ploty)

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    # print(left_curverad, 'm', right_curverad, 'm')
    
    left_lane_bottom = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
    right_lane_bottom = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
    distance_from_center = (1280 / 2 - (left_lane_bottom + right_lane_bottom) / 2) * xm_per_pix
```

The code is following the formulas in course lecture. For the distance_from_center, I calcudated the distance by using the middle point of the image (ie. width of image / 2) minus the middle point of the bottom of detected lanes.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in cell 13 and 14 in IPython notebook.  Here is an example of my result on the test image:

![result][result]

---

### Pipeline (video)

#### 1. Provide a link to your final video output. Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video_result.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here are some difficulties that I came across:

1. How to choose the thresholding method. I generated the output of different methods of sample image. Initally I thought just checking the S channel of HLS color space is enough. Later I noticed it works well on most cases, but it will fail on the road with bright color. Then I tried combination of single thresholding to get an acceptable solution for all cases.

2. In perspecitve tranform, how to define the source area. It should be transformed from a trapezoid to a rectangle, but how to find the vertex coordinates of the trapezoid is a little tricky. I opened the straight road image with image edit tool, mark the vertex coordinates manually, then fine tuned them in the code to make the result lanes appear parallel.

3. How to smooth the result between video frames. I noticed sometimes the green area in result video can be wiggle without smoothing. So I added a simple "smoother", as in cell 10 and the middle part of cell 11 in the IPython notebook. Basically the smoother will remember the latest 5 valid frames and return the average of the fitted lines every time. The result is considered as valid if the curvature radius of two lanes don't have significant difference.

I think my pipeline fail when there are very sharp turns, because the sliding window method seesm assume the lanes are mostly vertical. And also I think the thresholding method may not work very well when there are complicated shadow patterns on the road (like in the forest).

# IMAGE MOSAIC
--------------------

**Planys Technologies**


## __Overview__

_Image mosaicing(or image stitching) is the method of creating a Larger picture by stitching up smaller pictures. This method provides larger field of vision without compromising on resolution of the image._
_Two images are stitchable when we can identify enought number of(depending on the type of transformation assumption we are using)unique features._
### STEPS
1. Keypoint Identificaiton and description.
2. Matching the keypoints between two pictures.
3. Filtering out the good matches which are consistant.
4. Finding homography trasnformation matrix 

    Homogrpahy transformation needs a 3X3 matrix out of which one element is taken as 1 (scale factor). So total 8 variables has to be identified. This needs 4 points properly matched.
        So the theoratical minumum for finding out the homogprahy matrix is 4 points.
5. Warping the later picture to the former one according to the homography matrix.
6. Blending two images togoether to get a final bigger image.




## 1. Keypoint and Description

### 1.  AKAZE
_AKAZE_ (Accelerated - _KAZE_) is the speedup version of KAZE. It presents a fast multiscale feature detection and description approach for nonlinear scale spaces. It is both scale and rotation invariant.

### 2. ORB
_ORB_ (Oriented FAST and Rotated BRIEF), is an efficient alternative to _SIFT_ and _SURF_ algorithms(copyright protected). This algorithm is a combination of _FAST_  (Features from Accelerated Segment Test)and _BRIEF_ (Binary Robust Independent Elementary Features) algorithms. It is rotation invariant and robust to noise.

_**Although ORB is faster to compute, AKAZE shows a better compromise between speed
and performance than ORB. Since ROV struggles to detect feature points in underwater, AKAZE's reliable larger number of feature points is found favorable than ORB's fast calculations.**_

## 2. Matching
###  Brute Force Matcher
Brute-Force matcher is simple. It takes the descriptor of one feature in first set and is matched with all other features in second set using some distance calculation. And the closest one is returned. It is slower than its counterpart FLANN matcher(_not implemented_) which retruns approximate nearest neighbours. Since Brute-Force method tries every possible match and returns the closest one, it returns the best possible match.

_**Since the number of feature points are less in underwater images, Brute-Force is found out to be the better choice.**_

## 3. Good Match Selection
Every matched feature points need not to be correct or need not to be agreeing with each other. Good match selection will remove the outliers. Although the theoratical limit for the successful stitch is 4 good matches, a check is performed for atleast 10 good matches for better reliability.

## 4. Finding Homography
We are using Perspective transformation to find homography matrix. For perspective transformation, you need a 3x3 transformation matrix. Straight lines will remain straight even after the transformation. To find this transformation matrix, you need 4 points on the input image and corresponding points on the output image. Among these 4 points, 3 of them should not be collinear. RANSAC(Random Sample Consensus) is used to estimate the trasformation from all the good matches.



## 5. Warping
The second image has to be warped according to to the estimated homography before blending. the homography is used here.

## 6. Blending

Blending is the process of combining two images and making it seamless across the boundaries. Feather blending in OpenCV is used for this purpose.


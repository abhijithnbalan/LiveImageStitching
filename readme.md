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

### 2. ORB
ORB (Oriented FAST and Rotated BRIEF), is an efficient alternative to SIFT and SURF algorithms(copyright protected).

## 2. Matching
### 1. Brute Force Matcher

## 3. Good Match Selection

## 4. Finding Homography

## 5. Warping

## 6. Blending


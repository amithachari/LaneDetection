# LaneDetection
Lane Detection algorithm using openCV

Lane Detection - GEM Scenario
https://youtu.be/a3YowtNYIdM

Lane Detection - ROS Bags
https://youtu.be/anW6sVsAQzM

# Working
Perspective transform: Converted the image into Bird’s Eye View.

Color threshold: Threshold on color channels to find lane pixels. Coverted the
image from RGB space to HLS space and threshold the S channel.

Gradient threshold: Ran an edge detection on the image by applying a Sobel filter.
Combined binary image Combine the gradient threshold and color threshold image to get lane image.
Applied a Region of Interest mask to get rid of irrelevant background pixels and applied morphology function to remove noise.

Lane fitting: Extracted the coordinates of the centers of right and left lane from binary Bird’s Eye View image.
Fit the coordinates into two second order polynomials that represent right and left lane.

![image](https://user-images.githubusercontent.com/64373075/177688522-94b7586f-cd2a-4147-a9d3-856d61c2ae2c.png)

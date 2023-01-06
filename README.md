# Silhouetize
## Origins

Imagine you want to play around with some advanced image editing like
- applying gaussian or median blur
- binarizing the image with either a manual threshold, or using the Otsu, triangle, mean, or gaussian method
- performing some edge detection
- dilating or eroding a binary image
- find contours in an image and applying them as some sort of mask over the original image

This all sounds fun and games until you think of actually printing these contours onto a paper. <br>
Sure, you could just open the SVG in your favourite image viewer and press 'Print'. <br>
But where's the fun in that? And what if you wanted to cut the contours instead? <br>
Then you need a plotter/cutter device instead. Like the `Silhouette Cameo 4 Pro`, e.g.:

<img alt="An image showing the Silhouette Cameo 4 Pro device." height="100" src="https://www.creativamenteplotter.it/wp-content/uploads/2020/11/Silhouette-Cameo-4-pro-utilizzo.jpg" title="Silhouette Cameo 4 Pro" width="225"/>

## Solution (prototyping in the LearningLab)
[//]: # (TODO)
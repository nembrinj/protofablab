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

## UI Explained
In the following section we will explain all elements of the user interface. 
When first entering the application you are greeted by the following user interface.

<img src="./readmeImages/startup_screen.png" style="width: 75%;"/>

Taking a closer look at all the parts individually, we begin with the buttons.

<img src="./readmeImages/start_undo.png" style="width: 20%;"/>
<img src="./readmeImages/send_button.png" style="width: 20%;"/>

The first two buttons are Start and Undo. Start causes the image processing pipeline to begin and the newest image to be updated.
Undo causes to revert the last image...
The "Send To Silhouette" button. Once pressed it causes the most recent svg to get sent to the Silhouette machine. 
Upon which the machine draws the lines which can be seen in the svg. 

<img src="./readmeImages/ui_images.png" style="width: 25%;"/>

Above the send button you can see two images. The top image is the original 

Above the button you can see the most recent svg 


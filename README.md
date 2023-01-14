# Silhouetize

## Origins

Imagine you want to play around with some advanced image editing like

- applying gaussian or median blur
- binarizing the image with some threshold
- performing some edge detection
- dilate shapes in a binary image or make them smaller
- find contours in an image and maybe apply them as a mask over the original image

This all sounds fun and games until you think of actually printing these contours onto a paper. <br>
Sure, you could just open the SVG in your favourite image viewer and press 'Print'. <br>
But where's the fun in that? And what if you wanted to cut these contours into a birthday card instead? <br>
Then you will need a plotter/cutter device. Like the `Silhouette Cameo 4 Pro`, e.g.:

<img alt="An image showing the Silhouette Cameo 4 Pro device." height="100" src="https://www.creativamenteplotter.it/wp-content/uploads/2020/11/Silhouette-Cameo-4-pro-utilizzo.jpg" title="Silhouette Cameo 4 Pro" width="225"/>

## Solution (prototyping in the LearningLab)

[//]: # (TODO)

## UI Explained

In the following section we will explain all elements of the user interface.
When first entering the application you are greeted by the following user interface.

<img src="./readmeImages/startup_screen.png" style="width: 75%;"/>

Taking a closer look at all the parts individually, we begin with the buttons.

### Buttons

<img alt="The Start button in the user face applies the image processing pipeline." src="./readmeImages/start_undo.png" title="Start Button" width="20%"/>
<img alt="This button sends the current SVG to the Silhouette device for printing/cutting." src="./readmeImages/send_button.png" title="Send to Silhouette Button" width="20%"/>

The first two buttons on the top beside the pipeline options are **Start** and **Undo**. <br>
**Start** applies the selected image processing pipeline and updates the current image and SVG accordingly. <br>
**Undo** reverts the last pipeline to the last image. <br>
**Send To Silhouette** is situated below the images/SVGs. Once pressed, it sends the current SVG to the Silhouette
machine for printing/cutting.

<img alt="The image/SVG stack of pipeline results." src="./readmeImages/ui_images.png" title="Image results" width="20%"/>
Inbetween the pipeline options and the `Send to Silhouette` button you initially see 2 images.
The more pipelines you apply, the more images you will see.
Currently, a stack of 5 states are saved. This may be adapted manually in the source code. <br>
The top row shows the last images, whereas the bottom row shows the corresponding contours/SVG.


## NewPlayer

NewPlayer is a video manipulation program that can swap the face of a person in a video with the face of a different person in an image selected by the user.
The project is written in C++ using wxWidgets for the GUI. It uses OpenCV and dlib for image processing. This project was created as an exercise to learn about wxWidgets, OpenCV and the dlib library.

## Description
The program contains multiple tabs that demonstrates the video manipulation process in order to swap the face.
This includes the "Original Video", "Facial Landmarks", "Convex Hull", "Delaunay Triangulation", and the "Face Swapped" tabs.
<br>
The facial landmarks process uses the dlib library to retrieve 68 points on the face.
These points are then used to create a convex hull of the face using an openCV function.
The facial landmarks are also used to determine the Delaunay triangulations inside the face.
This process is done for both the source image and destination video.
We then swap corresponding triangles with triangles from the source image.
This results in an image with a swapped face.

## Demonstration
We use a video of Joe Biden and swap his face with an image of Tom Cruise.

<img src="https://github.com/kim2552/NewPlayer/blob/main/assets/tomcruise.jpg" alt="tomcruise" width="400"/>

<img src="https://github.com/kim2552/NewPlayer/blob/main/assets/example_gif.gif" alt="joebiden" width="400"/>

## Resources
Most of the image manipulation process was developed based on this article: https://learnopencv.com/face-swap-using-opencv-c-python/

# Identifying-driving-lanes-
Identifying driving lanes using "Hough Transform" theory

## Installation packages

```python
import cv2
import numpy as np
```

## General code review

- First i took a single frame from the wanted video in order to analyze on.

<img src="https://user-images.githubusercontent.com/101269937/190387214-c057423b-1352-4f03-af55-8d9f55fd5d20.jpg" width="250" height="200">

- Before every CV project, it's easer to working on binary image. In this project i used canny edge and worked on gray scale.


<img src="https://user-images.githubusercontent.com/101269937/190392105-5ff50dca-6441-48c1-974a-6ce650abe77b.jpg" width="250" height="200">

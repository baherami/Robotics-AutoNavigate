## Project: Search and Sample Return
[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

[image1]
The goal of this project is to provide simple perception, decision making and actuation for autonoumous navigation of a robot in a simulator. The simulator is written in Unity.
This project has been tested with the following specs:


* OS:Windows 10
* RAM: 16GB
* Sim Screen: 800*600,Good,Display1,Windowed
* FPS output had the following values: None,15,31,29,31,29,30,26,...  

This demo contains codes that are fully tested and verified on Windows 10.

---

### Installation

* Download the simulator for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip) or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip). 
* Install [Jupyter Notebook](http://jupyter.org/)
* Create the environment and install its dependencies from [here](https://github.com/ryan-keenan/RoboND-Python-Starterkit/blob/master/doc/configure_via_anaconda.md).
* From terminal activate environment.


### Testing methods
There is a notebook in the project to validate the perception phase based on captured images from simulator. The test dataset includes captured images from the simulator and the output is a short video to demonstrate how these methods work.
To run the notebook :
* From the activated terminal and in the project root, run: jupyter notebook
### Running in the autonomous mode.
* Execute Python in the active environment : `python code/drive_rover.py`
* Run the simulator and choose autonomous mode.

---

### Notebook's Notes:
**test dataset**

Captured images are in `test_dataset`.

**color_thresh**

Recognizing objects  is done with usual RGB threshholding and [opencv HSV color threshing method](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)

**process_image**

In this function the output mosaic image is rendered with mapping and rover cam view and threshed rover cam view.

### Autonomous Navigation and Mapping 

**Perception**

Same logic is used in perception.py for processing images.

**Decision**

In decision.py, a simple mechanism is used to collect rock samples. Two new modes added to the rover for this purpose.

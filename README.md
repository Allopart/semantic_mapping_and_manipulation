# Generalized Framework for the Parallel Semantic Segmentation of Multiple Objects and Posterior Manipulation

[![Alt text for your video](https://img.youtube.com/vi/HLzFLMktIYc/0.jpg)](https://www.youtube.com/watch?v=HLzFLMktIYc)


The end-to-end approach presented in this repo deals with the recognition, detection, segmentation and grasping of objects, assuming no prior knowledge of the environment nor objects. The contributions of the paper are as follows: 1) Usage of a trained Convolutional Neural Net (CNN) that recognizes up to 80 different classes of objects in real time and generates bounding boxes around them. 2) An algorithm to derive in parallel the pointclouds of said regions of interest (RoI). 3) Eight different segmentation methods to remove background data and
noise from the pointclouds and obtain a precise result of the semantically segmented objects. 4) Registration of the objects’ pointclouds over time to generate the best possible model. 5) Utilization of an algorithm to detect an array of grasping positions and orientations based on the geometry of the objects model. 6) Implementation of the system on the humanoid robot MyBot, developed at the RIT Lab at KAIST. 7) An algorithm to find the bounding box of the objects model in 3D to then create a collision object and add it to the octomap. The collision checking between robots hand and the object is removed to allow grasping using the MoveIt libraries. 8) Selection of the best grasping pose for a certain object, plus execution of the grasping movement. 9) Retrieval of the object and moving it to a desired final position.

## Getting Started
To understand what the code does, it is highly recommended that you read the paper which describes in detail all the parts of the framework.

### Prerequisites

agile_grasp
dn_object_detect
darknet
mybot


### Installing

## Running the tests

## Built With

* [ROS Indigo](http://wiki.ros.org/indigo)


## Authors

* **Adrian Llopart** - *Initial work* - [Allopart](https://github.com/Allopart)

## Acknowledgments

* RIT Lab in KAIST for supplying humanoid robot


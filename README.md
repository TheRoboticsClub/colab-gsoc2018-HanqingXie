# colab-gsoc2018-HanqingXie

## Project Description
* The main idea of this project is to introduce the OMPL (Open Motion Planning Library) into JdeRobot-Academy, in a new robot navigation exercise. 

* Develop a new exercise and ompl's solutions using different path planning algorithms of an autonomous wheeled robot or drone which moves along a known scenario in Gazebo.

* You can refer to the [project wiki](https://jderobot.org/Club-hanqingxie) for more details.

## Overview
This project inculdes three exercises:
* NavigationOmpl Exercise
* Autopark Exercise
* Navigation3DOmpl

## Dependency
This project requires the follow dependency:
* jedrobot
* python2.7
* [OMPL](http://ompl.kavrakilab.org/)

### install ompl
[Download the OMPL installation script](http://ompl.kavrakilab.org/install-ompl-ubuntu.sh). First, make the script executable:

```chmod u+x install-ompl-ubuntu.sh```

Next, there are three ways to run this script:

```./install-ompl-ubuntu.sh --python will install OMPL with Python bindings```

## NavigationOmpl Exercise

The objective of this practice is to learn the use of ompl to implement a local navigation in the taxi.
1. Run Gazebo simulator

    ```gazebo cityLarge.world```

2. Running the practice and the user interface:

    ```python2 NavigationOmpl.py taxiMap.conf teleTaxi_conf.yml```

## Autopark Exercise

The goal of this exercise is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must find a parking space and park properly.

1. Run Gazebo simulator:

    ``` gazebo autopark.world```

2. Running the practice and the user interface: 

    ``` python2 autoparkOmpl.py autopark_conf.yml```


## Navigation3DOmpl Exercise

In this practice we will learn the use of ompl to implement a local navigation algorithm in the quadricopters.
For this practice a world has been designed for the Gazebo simulator. This world has a 3D model of the AR.Drone and a indoor environment. 

1. Copy the inHouse model into jderobot gazebo models

	``` cp -r ../models/inHouse /root_jderobot/share/jderobot/gazebo/models/```

2. Run Gazebo simulator:

	``` gazebo ardrone-inHouse.world```

3. Running the practice and the user interface:

	``` python2 ./Navigation3dOmpl.py ardrone_conf.yml```
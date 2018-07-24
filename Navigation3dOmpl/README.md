		POSITION CONTROL EXCERSISE
		==========================

In this practice we will learn the use of PID controllers to implement a local navigation algorithm in the quadricopters.
For this practice a world has been designed for the Gazebo simulator. This world has a 3D model of the AR.Drone and 5 beacons arranged in cross mode. The intention is to make the drone do the following route: the first beacon to visit is the one to the left of the drone, the next will be the one in front, then you will have to go to the one that was located to the left of the initial position, then the one in the back, and finally to the one in front, in the most distant position. Finally we will make it return to the initial position and land.

///////////////////////////////////////////////////////////////////
 			E X E C U T I O N 
///////////////////////////////////////////////////////////////////

Copy the inHouse model into jderobot gazebo models
$ cp -r inHouse ${CMAKE_INSTALL_PREFIX}/share/jderobot/gazebo/models/

In a terminal launch the gazebo simulator:
$ gazebo ardrone-inHouse.world

In other terminal launch the position_control component with your algorithm:
$ python2 ./Navigation3dOmpl.py ardrone_conf.yml


///////////////////////////////////////////////////////////////////

### API

* self.pose.getPose3d().x - returns the position values ​​of the drone in X axis
* self.pose.getPose3d().y - returns the position values ​​of the drone in Y axis
* self.pose.getPose3d().roll, self.pose.getPose3d().pitch, self.pose.getPose3d().yaw: returns the rotation values ​​of the drone in space.
* self.camera.getImage().data: returns the image captured by the active camera of the drone (frontal or ventral).
* self.cmdvel.setVX(velx): set linear speed of the drone.
* self.cmdvel.setVY(vely): set linear speed of the drone.
* self.cmdvel.sendVelocities(): send set velocities to the drone.
* self.cmdvel.sendCMDVel(self,vx,vy,vz,ax,ay,az): sends linear and angular speed commands to the drone.
* self.extra.toggleCam(): changes the drone's active camera (frontal or the one below).
* self.extra.takeOff(): Takeoff of the drone.
* self.extra.land(): landing of the drone.


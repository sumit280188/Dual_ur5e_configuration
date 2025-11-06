### Launching

	ros2 launch bim_object_sequencer sequencer.launch.py

### Nodes

### 1) spawner
loads the IFC file, makes meshes, and publishes them into the scene (also adds a floor). It can spawn an object by index, attach it to the robot, or detach it back.

### 2) sequencer
follows the montage order from the JSON. It tells the spawner which object to spawn next, and waits until that object is detached before moving on.

### 3) attached_base
a helper that loads some chosen IFC elements and adds them as fixed background objects in the scene. This is to add the wall and the bottom pipes, so that we can pickupandplace onto those already fixed pipes. 

# ROSProject
 
Hello, this is the project for the ROS LoCoBot project in thecontruct.ai.
Due to rubric metrics, we have been asked to link the code as a repo as well, hence this repo has been made and you're currently reading the ReadME.

WARNING! When committing this code (which was previously only built in the workspace of theconstruct.ai) we were prompted with the warning that we couldn't properly commit embedded git repo's.
OUR code can be found within the files, as well as the hand-in on canvas, to say the code directly from this repo works, is unclear. 

WARNING #2! The code for both the scripts subscr_camera.py and loco_movements.py have been tested with python <script_name.py> in the terminal. Rosrun is also a good option, but keep in mind that not every terminal might process it as well as when using the 'python' command.

As for the content itself. 
- In path camera_detection/scripts/subscr_camera.py you will find the code for the camera on the LoCoBot to detect the coloured objects in front of it. When running this code, the camera will indefinitely cycle from going left to right while observing its environment. 

- In path arm_movements/scripts/loco_movements.py you will find the code for the interbotix arm to move towards the detected opbjects. In this case, you should see it first reachinng for the red cube. After hovering above the objerct with the end effector, it will return to its home position before repeating the cycle for a different object.


# Robot-Aided-Drafter
*Robot Workflow*

The files hosted in this repository are used to implement the Robot Aided Drafter. 

More in-depth details regarding the project can be found here: https://charith-sunku.github.io/Robot-Aided-Drafter/

## Software Workflow
**Image Parsing** - Our Python script takes .png and .jpg files as inputs. Using OpenCV and our inverse kinematics algorithm, outputs a list of joint angles to draw the image.

**Output Verification** - Using MATLAB, we verify the the fidelity of the processed drawing by simulating the robots motion and tool path.

**Robot Motion** - Our C based codebase interfaces with low-level hardware to actuate the robot by reading the joint angles outputted from Python. 

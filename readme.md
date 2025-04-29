# Directory Description

<ol>

<li>
<code>./UUV</code> This folder mainly contains the Unmanned Underwater Vehicle (UUV) model, camera parameters, and launch scripts.
<ol>
<li>
<code>Config.json</code> is the parameter file for the stereo camera.  
Changing the <code>TypeID</code> parameter allows you to modify the sensor type. <code>TypeID=1</code> corresponds to RGB images, <code>TypeID=2</code> to grayscale images, and <code>TypeID=3</code> to depth images.
</li>
<li>
<code>UUV.params</code> are the PD control parameters for the UUV, used in Hardware-in-the-Loop (HITL) simulations.
</li>
<li>
<code>px4_fmu-v6c_default.px4</code> is the HITL flight controller firmware for the UUV targeting the PX4 FMU v6c hardware. For other versions, refer to the firmware compilation tutorial below.
</li>
<li>
<code>UUVModel.dll</code> is the dynamics model of the UUV, developed using Simulink and compiled for use in CopterSim software. This file should be in the same directory as the one-click launch script.
</li>
<li>
<code>UUVModel_HITL.bat</code> is the one-click launch script for HITL simulation with UE4.
</li>
<li>
<code>UUVModel_SITL.bat</code> is the one-click launch script for Software-in-the-Loop (SITL) simulation with UE4.
</li>
<li>
<code>UUVModel_HITLUE5.bat</code> is the one-click launch script for HITL simulation with UE5.
</li>
<li>
<code>UUVModel_SITLUE5.bat</code> is the one-click launch script for SITL simulation with UE5.
</li>
</ol>
</li>

<li>
<code>./Demo</code> This folder mainly contains the UUV control demo code.
<ol>
<li>
<code>UUVAttCtrlCamera.py</code> is a sample program for the camera (does not start ROS publishing).
</li>
<li>
<code>UUVAttCtrlPath.py</code> is a sample program for UUV control.
</li>
</ol>
</li>

<li>
<code>./euroc_uuv</code> This folder mainly contains configurations for topics used in Vins subscriptions. To run the fusion localization, this configuration is required.
</li>

<li>
<code>./FusionLocation</code> This folder contains the fusion localization code for the UUV.
<ol>
<li>
<code>Config.json</code> is also the parameter file for the stereo camera. It is the same as the one in the UUV directory, and this file needs to be in the same directory as the image-capturing code.
</li>
<li>
<code>kf.py</code> is the Kalman filter code.
</li>
<li>
<code>UUVAtt_server.py</code> is the remote control program for the UUV (can be run remotely).
</li>
<li>
<code>relocate.py</code> is the relocalization program for VINS.
</li>
<li>
<code>ropeInfo_generator.py</code> is the rope localization program.
</li>
<li>
<code>path_fusion.py</code> is the position fusion program.
</li>
<li>
<code>oneKeyScript.sh</code> is the one-click startup script for all programs.
</li>
</ol>
</li>

</ol>

# Starting Software-in-the-Loop (SITL)

<ol>
<li>
Run the following script as administrator: <code>./UUV/UUVModel_SITL.bat</code>, and input <code>1</code> to create an aircraft. This script will open CopterSim, QGroundControl, and RflySim3D.
</li>
<li>
Run the example programs located in the Demo folder.
</li>
</ol>

# Starting Hardware-in-the-Loop (HITL)

<ol>
<li>
On the host machine running the simulation platform, plug in the flight controller hardware (with firmware already flashed) via USB.
</li>
<li>
Run the following script as administrator: <code>./UUV/UUVModel_HITL.bat</code>, and input the corresponding COM port number when prompted. The port number will be displayed in the command line window. This script will open CopterSim, QGroundControl, and RflySim3D.
</li>
<li>
Run the example programs located in the Demo folder.
</li>
</ol>

# Starting Fusion Localization

<ol>
<li>
Prepare an Ubuntu virtual machine and install Python 3.8.
</li>
<li>
Download and compile <a href="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion" title="Vins-fusion">VINS-Fusion</a>, using the configurations provided in the <code>./euroc_uuv</code> folder.
</li>
<li>
Copy the <code>./FusionLocation</code> directory to the virtual machine.
</li>
<li>
Copy the <code>RflySimSDK</code> folder from the platform API directory to the virtual machine, and run <code>ReLabPath.py</code> to load the Python interface provided by the platform (this must be done after each platform update).
</li>
<li>
Run either <code>./UUV/UUVModel_SITLUE5.bat</code> or <code>./UUV/UUVModel_HITLUE5.bat</code> as administrator to start the simulation platform.
</li>
<li>
On the virtual machine, run <code>oneKeyScript.sh</code> to start the fusion localization process in one click.
</li>
</ol>

Note: All the above steps require the RflySim platform. For more information and detailed tutorials, please visit: <a href="www.rflysim.com" title="Rflysim">www.rflysim.com</a>
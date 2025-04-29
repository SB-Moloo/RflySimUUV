<h1>目录描述</h1>

<ol>

<li>
<code>./UUV</code> 该文件夹主要包含无人潜航器的模型、相机参数和启动脚本等。
<ol>
<li>
<code>Config.json</code> 是双目相机的参数文件。
通过修改<code>TypeID</code>参数可以实现修改传感器类型。<code>TypeID=1</code>为RGB图，<code>TypeID=1</code>为灰度图，<code>TypeID=1</code>为深度图。
</li>
<li>
<code>UUV.params</code> 是无人潜航器的PD控制参数，用于硬件在环仿真。
</li>
<li>
<code>px4_fmu-v6c_default.px4</code> 是无人潜航器的硬件在环飞控固件，用于硬件在环仿真。适用于PX4的6c型号，如果型号不对应，也可以见后文的编译固件教程。
</li>
<li>
<code>UUVModel.dll</code> 是无人潜航器的动力学模型文件，由Simulink编写并生成，用于CopterSim软件。该文件应与一键启动脚本在同一目录下。
</li>
<li>
<code>UUVModel_HITL.bat</code> 是无人潜航器的硬件在环（UE4）的一键启动脚本。
</li>
<li>
<code>UUVModel_SITL.bat</code> 是无人潜航器的软件在环（UE4）的一键启动脚本。
</li>
<li>
<code>UUVModel_HITLUE5.bat</code> 是无人潜航器的硬件在环（UE5）的一键启动脚本。
</li>
<li>
<code>UUVModel_SITLUE5.bat</code> 是无人潜航器的软件在环（UE5）的一键启动脚本。
</li>
</ol>
</li>

<li>
<code>./Demo</code> 该文件夹主要包含无人潜航器的控制Demo代码。
UUVAttCtrlCamera.py
</li>
<ol>
<li>
<code>UUVAttCtrlCamera.py</code> 是相机的示例程序（没有启动ROS发布）。
</li>
<li>
<code>UUVAttCtrlPath.py</code> 是潜航器控制的示例程序。
</li>
</ol>
</li>

<li>
<code>./euroc_uuv</code> 该文件夹主要包含Vins订阅的话题的配置，运行融合定位，需要使用该配置。
</li>

<li>
<code>./FusionLocation</code> 该文件夹主要包含无人潜航器的融合定位代码。
<ol>
<li>
<code>Config.json</code> 也是双目相机的参数文件。和UUV目录中的一样，该文件需要和启动取图的代码在同一目录下。
</li>
<li>
<code>kf.py</code> 是卡尔曼滤波代码。
</li>
<li>
<code>UUVAtt_server.py</code> 是无人潜航器的远程控制程序（可以远程运行）。
</li>
<li>
<code>relocate.py</code> 是Vins的重定位程序。
</li>
<li>
<code>ropeInfo_generator.py</code> 是绳索定位程序。
</li>
<li>
<code>path_fusion.py</code> 是定位融合程序。
</li>
<li>
<code>oneKeyScript.sh</code> 是所有程序的一键启动脚本。
</li>
</ol>
</li>

</ol>

<h1>软件在环启动</h1>
<ol>
<li>
以管理员方式运行：<code>./UUV/UUVModel_SITL.bat</code>，并输入<code>1</code>来创建一架飞机。该脚本会打开CopterSim、QGroundControl和RflySim3D。
</li>
<li>
运行Demo文件夹中的示例程序。
</li>
</ol>

<h1>硬件在环启动</h1>
<ol>
<li>
在准备运行仿真平台的主机中，使用USB插入刷好固件飞控硬件。
</li>
<li>
以管理员方式运行：<code>./UUV/UUVModel_HITL.bat</code>，并输入对应的端口号，端口号会在命令行窗口中提示。该脚本会打开CopterSim、QGroundControl和RflySim3D。
</li>
<li>
运行Demo文件夹中的示例程序。
</li>
</ol>

<h1>融合定位启动</h1>
<ol>
<li>
准备一台Ubuntu虚拟机，安装好Python3.8。
</li>
<li>
下载并编译好<a href="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion" title="Vins-fusion">vins</a>，使用<code>./euroc_uuv</code>中的配置。
</li>
<li>
将<code>./FusionLocation</code>目录复制到虚拟机。
</li>
<li>
将平台API目录中的<code>RflySimSDK</code>文件夹复制到虚拟机中，并运行<code>ReLabPath.py</code>脚本来加载平台提供的Python接口（每次更新平台需要重新加载）。
</li>
<li>
以管理员方式运行：<code>./UUV/UUVModel_SITLUE5.bat</code>或<code>./UUV/UUVModel_HITLUE5.bat</code>来启动仿真平台。
</li>
<li>
在虚拟机中运行<code>oneKeyScript.sh</code>来一键启动融合定位程序
</li>
</ol>

Note: 以上需要运行在RflySim平台，平台下载和更详细的教程请见：<a href="www.rflysim.com" title="Rflysim">www.rflysim.com</a>
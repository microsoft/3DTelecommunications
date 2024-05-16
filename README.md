# 3D Telemedicine

> This is the open source repository for the 3D Telemedicine system created by Microsoft Research Special Projects Group: https://www.microsoft.com/en-us/research/project/3d-telemedicine/
[![Watch the video](https://img.youtube.com/vi/ml4Dv5CWC7s/maxresdefault.jpg)](https://youtu.be/ml4Dv5CWC7s)

## System Requirements
The typical system layout for 3D Telemedicine is to have one machine running Fusion, and a separate machine running Render and Viewer.  The pods all run their own copy of the K4AToFusion code and should be networked into the same network as Fusion.  Control panel can be run on the Fusion PC, the Render PC, or a separate PC or tablet.

### Minimum Operating Requirements (Fusion)
Windows 11
CUDA 11.7
NVidia GPU 2080Ti or larger
10 Gigabit ethernet adapter
OpenCV 4.5.4
We use OpenCV with CUDA libraries.  This means you cannot just use the binaries available directly on the website, although there are typically additional sites that provide OpenCV with CUDA built in.
Currently our project requires the opencv_contrib folder also be checked out of GIT and compiled as part of OpenCV.  We hope to remove this dependency in the future.


### Minimum Operating Requirements (Render)
Windows 11
CUDA 11.7
NVidia GPU 2080Ti or larger
1 Gigabit ethernet adapter

## Building from Source
## 3rd Party Components (Windows)
This system relies on the following open-source 3rd party components:
| Component | Version | Link |
| --------- | ------- | ---- |
| boost | 1.72.0 | https://www.boost.org/users/download/ |
| ceres-solver | 1.9.0 |https://ceres-solver.googlesource.com/ceres-solver|
| cppzmq | 4.7.1 | https://github.com/zeromq/cppzmq |
| freeglut | 3.0.0 | https://freeglut.sourceforge.net/ |
| glew | 1.7.0 |http://glew.sourceforge.net/ |
| libjpeg | 9c | https://github.com/winlibs/libjpeg |
| jsoncpp | 1.8.1 | https://github.com/open-source-parsers/jsoncpp |
| libzmq | 4.2.0 | https://github.com/zeromq/libzmq |
| LZ4 | 1.8.3 | https://github.com/lz4/lz4 |
| oglplus | 0.72.0 | https://github.com/matus-chochlik/oglplus |
| OpenCV | 4.5.4 (CUDA) | https://github.com/opencv/opencv |
| qhull | 2011.1 | http://www.qhull.org |
| suitesparse-metis | 1.5.0 | https://github.com/jlblancoc/suitesparse-metis-for-windows.git |
| VXL | 2.0.2 | https://github.com/vxl/vxl.git |
| xerces | 3.1.1 | https://github.com/apache/xerces-c/ |
| zlib | 1.2.5 | https://www.zlib.net/ |
| CUDA | 11.7 | https://developer.nvidia.com/cuda-11-7-1-download-archive? |
| Visual Studio | 2019 | https://visualstudio.microsoft.com/downloads/ |


The default configuration expects these dependencies to be placed in a Dependencies\ folder inside the root of the repo.  You can change this location by modifying the definition of Peabody_Dependency_Dir in PeabodyConfigurationMacros.props, or modifying only a specific component's location in its Peabody.x64.[componentName].props file.

## 3rd Party Components (Linux)
| Component | Version | Link |
| --------- | ------- | ---- |
| libk4a | 1.4.1 | https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download |
| ZeroMQ | 4.3.4 | https://github.com/zeromq/libzmq/ |
| CppZMQ | | https://github.com/zeromq/cppzmq |
| OpenCV | 4.5.4 (CUDA) | https://github.com/opencv/opencv |
| CUDA | 11.x | Bundled in Jetpack OS Image |



## System Components 
This repo includes source code to compile the following systems:
### Fusion
Fusion is the main component that creates a watertight, volumetric mesh in real time from any number of RGB+D streams.  The textured volume and raw (or jpeg compressed) RGB images are transmitted to any number of render clients.  
It is based off of the Fusion4D algorithm https://www.microsoft.com/en-us/research/publication/fusion4d-real-time-performance-capture-challenging-scenes-2/
Fusion is a C++ and CUDA project, and is known to work with CUDA 11.4+.  It uses Boost ASIO for transmmitting data.
### Render
Render takes a volumetric mesh from the fusion system and textures it with the RGB images provided in the same data stream from fusion, and transmits the textured model over the network to any number of viewer clients.  
It is a C# Unity project, and is known to work with Unity 2019.4.14f1
It is also dependent on the C++ Rendering Client DLL for transmitting data.  It uses Boost ASIO for transmitting data.
### Viewer
Viewer subscribes to a given Render stream and presents the textured 3D model to the user.  It provides views of the 3D model and the 2D cameras.  It also includes drawing tools and capture tools for manipulating the model.
It is a C# Unity project, and is known to work with Unity 2019.4.14f1.  It uses Boost ASI for transmitting data. 
### Control Panel
The control panel software orchestrates all of the components of the 3D Telemedicine system.  It provides one interface to start a calibration session, start a broadcast (live) session, modify configuration values, transmit configuration to the various components, monitor status and version of all components in the system, and check for updated components from an Internet-accessible source.
It is a C# Unity project, and is known to work with Unity 2019.4.14f1. It utilizes the zeroMQ networking library for all communication.
### Calibration Software
The calibration software will take a number of synchronized input mkv videos from cameras in the system (Azure Kinect) and output a calibration file that provides world coordinates of all cameras in the system.   It is dependent on the PAI calibration system available at this NuGet repository: 
It is a C++ project.  It uses ZeroMQ for transmitting progress updates to the Control Panel.
### K4ARecorder
This is a port of the K4ARecorder provided in the Azure Kinect SDK that integrates it into our system and communicates with the Control Panel.  It performs the same functions as K4ARecorder and is dependent on that library.  Recorded videos are transmitted from the camera units to the fusion machine via scp.
### K4A To Fusion
K4AToFusion is the ingest code that pulls images from an Azure Kinect and transmits them to a Fusion server.  It is written to run on an Nvidia Jetson Nano (Linux)
It is written in C++ and uses the ZeroMQ library for communication with the Control Panel, and Boost IO libraries for streaming image data.
### Azure Kinect Launcher Daemon
Azure Kinect Launcher Daemon is the Linux system daemon that monitors the status of the Kinect-attached Jetson Nanos and communicates with the Control Panel.  The Control Panel uses this daemon to launch the K4A To Fusion software, K4ARecorder software (for calibration), check version information, upload new configuration files and software versions, and receive updates about current system and application status.
It is written in C++ and uses the ZeroMQ library for communication.
### 3DTM Launcher Service
3DTM Launcher Service is the Windows service that runs on Fusion and Render machines and communicates with the Control Panel.  The Control Panel uses this service to launch Render, Fusion, and the Calibration Software.  It also uses this service to check software versions, transmit configuration data and new software versions, and receive updates on current system and application status.
It is written in C++ and uses the ZeroMQ library for communication.
### Kinect Nano Communicator Service
In certain configurations where the camera system may be on a separate subnet from the control panel, the Kinect Nano Communicator Service can act as a router to route messages from the Azure Kinect Launcher Daemon and K4A To Fusion components of each camera unit up to the control panel on a separate subnet.

## Setup [Windows components]
Check out the repository to a location of your choice, we'll refer to it as [3dtmroot]

Create the folder for the dependencies, [3dtmroot]\Dependencies

Download the dependencies and install them into the Dependencies folder

### Build (Fusion)
Fusion is built using the DeformableFusion solution in the DeformableFusion folder of the repository.  It should be built in Release, x64 configuration.  Debug currently does not compile. The DeformableFusion solution contains a number of helper libraries that will be built first, and the Fusion executable is built as part of the LiveFusionDemo-MultiView project, which is selected as the startup project.


## Setup [Linux components]
The linux components (Azure Kinect Launcher Daemon, K4A To Fusion, K4ARecorder) are built for the ARM64 architecture of the Nvidia Jetson Nano.  Our build system uses the Ubuntu 18.04 image from the Nvidia Jetson website that includes CUDA.

Check out the repository to a location of your choice, we'll refer to it as [3dtmroot]

Clone the Azure Kinect SDK: https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git

Install the additional required build packages:
- Azure Kinect libraries - https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions
- Ncurses
- SoundIO (required for Azure Kinect)
- Boost - System
- Boost - IO Streams
- ZMQ
- ProcPS
- LZ4
- JSONCpp
- ZeroMQ 4.3.4
- cppzmq
- OpenCV 4.5.4
  
```bash
sudo apt update && sudo apt install -y libk4atools libk4a1.4  libk4a1.4-dev libncurses-dev libsoundio-dev libboost-system-dev libboost-iostreams-dev libzmq3-dev libprocps-dev liblz4-dev libjsoncpp-dev
```
The system requires zeromq 4.3.4 with drafts enabled, which is not available in the Jetson Ubuntu 18.04 image.  So to do this:
```bash
ZMQ_VERSION=4.3.4
PREFIX=/usr/local
wget https://github.com/zeromq/libzmq/releases/download/v${ZMQ_VERSION}/zeromq-${ZMQ_VERSION}.tar.gz -O libzmq.tar.gz
tar -xzf libzmq.tar.gz
cd zeromq-${ZMQ_VERSION} 
./configure --prefix=${PREFIX} --enable-drafts
make -j && make install
```
You can install cppzmq from here: https://github.com/zeromq/cppzmq

You'll also need OpenCV 4.5.5.  Here's a great tutorial to get it built on the nano: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html

Now set up the paths correctly in the cmakelists.txt

### Build (Linux)
You can build all of the linux projects at the same time by
```bash
cd [3dtmroot]
mkdir build
cd build
cmake ..
make
sudo make install
```

### Reminder
You might be tempted to update your build environment to a newer version of Ubuntu or other libraries, but remember that most libraries will be dynamically linked, not bundled in the binary, so they won't run on a Jetson Nano, which only has Ubuntu 18.04 as the latest OS.


## Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.opensource.microsoft.com.

When you submit a pull request, a CLA bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., status check, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Trademarks

This project may contain trademarks or logos for projects, products, or services. Authorized use of Microsoft 
trademarks or logos is subject to and must follow 
[Microsoft's Trademark & Brand Guidelines](https://www.microsoft.com/en-us/legal/intellectualproperty/trademarks/usage/general).
Use of Microsoft trademarks or logos in modified versions of this project must not cause confusion or imply Microsoft sponsorship.
Any use of third-party trademarks or logos are subject to those third-party's policies.

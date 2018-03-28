Installation
=============

We use catkin to build our tools. So make sure that you have a working catkin workspace.

Prerequisites & Dependencies
----------------------------
The roboy_communication_cognition package. It can be found in the roboy_communication
repository (https://github.com/Roboy/roboy_communication). Best practice is to store the
repository in the source folder of your catkin workspace.

libgstreamer1.0-0, libgstreamer1.0-dev, libgstreamer-plugins-base1.0-0, libgstreamer-plugins-base1.0-dev,
gstreamer1.0-plugins-ugly & gstreamer1.0-plugins-good, fftw3, libgflags-dev
To have them installed, you need to simply do
``sudo apt install <package_name>``

In addition doa_estimation is just build and executable on a Raspberry Pi, since
it uses it's Libraries and the current version just works with a Matrix Creator and a Pi.

In the end it is mandatory for you to be the superuser when you're using the audio programs.

Using command line
------------------

Clone the audio repository into the source folder of your catkin workspace.
``git clone https://github.com/Roboy/roboy_audio /path/to/catkin_workspace/src/roboy_audio``

Navigate to your catkin workspace.
``cd /path/to/catkin_workspace``

Make the packages. The available packages are: doa_estimation, roboy_audio_stream
``catkin_make --pkg <package_name>``

Have fun with roslaunch.
Start the direction of arrival estimation
``roslaunch doa_estimation matrix_doa.launch``

Capture and stream audio from the default audio source
``roslaunch roboy_audio_stream capture.launch``

Capture and stream audio from the 8 Matrix Creator microphones
``roslaunch roboy_audio_stream capture_matrix_mics.launch``

Start the stream receiver template node
``roslaunch roboy_audio_stream receive_py.launch``


Using IDE (Clion)
----------------------------------

Clone the audio repository.
``git clone https://github.com/Roboy/roboy_audio``

Now open the CMakeLists.txt from the package that you want to edit.

# RPi2Cam-Vid
Purpose -- a stripped down version of rpicam-vid that supports 2 concurrent cameras, PIP overlay and multiple output files and streams.
by stripped down I mean all no other output than libav, removed post image processing, preview and all other utilities of rpicam-apps. 
* libcamera development package
sudo apt install -y libcamera-dev 
* libav development packages
sudo apt install libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev
* git package
sudo apt install -y git

* boost and other packages used 
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
* meson build enviroment packages
sudo apt install -y meson ninja-build
*copy the repository 
https://github.com/wkeeling63/RPi2Cam.git
*build program
cd to the cloned repostitory
* one time meson setup
meson setup build -Denable_libav=enabled 
*compile 
meson compile -C build
*compile and install
sudo meson install -C build
* might be need it of shared library not found 
sudo ldconfig

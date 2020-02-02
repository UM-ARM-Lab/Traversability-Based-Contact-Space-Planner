Traversability-based Contact Space Planner
=============

Traversability-based contact space planner code release. Given environment specified as a set of polygonal surface and a robot model, the code generate contact sequence for a humanoid robot. We provide an example with the [Escher humanoid robot](https://icat.vt.edu/projects/2015-2016/major/escher-humanoid-robot.html) model. The code is written in Python 2.7 and tested in Ubuntu 14.04 with ROS Indigo.

Installation
------------
* Install [OpenRAVE](https://github.com/rdiankov/openrave), [Installation Guide](https://scaron.info/teaching/installing-openrave-on-ubuntu-14.04.html) (Tested Commit: 7c5f5e27eec2b2ef10aa63fbc519a998c276f908)
* Install [ROS](http://wiki.ros.org/indigo/Installation/Ubuntu)
* Install `srdfdom` and `tinyxml2`: `sudo apt-get install ros-indigo-srdfdom libtinyxml2-dev`
* Install `Eigen` and `NEWMAT`: `sudo apt-get install libeigen3-dev libnewmat10-dev`
* Install [cddlib](https://inf.ethz.ch/personal/fukudak/cdd_home/) (Tested Version: cddlib-094h):

Get cddlib from ftp:
```
wget ftp://ftp.math.ethz.ch/users/fukudak/cdd/cddlib-094h.tar.gz
tar -xvf cddlib-094h.tar.gz
```
or from github:
```
git clone https://github.com/cddlib/cddlib/tree/0.94h
```
Install cddlib and dependencies:
```
sudo apt-get install libgmp3-dev
cd cddlib-094h
./configure
make
sudo make install
cd /usr/local/include
sudo mkdir cdd
sudo mv cdd_f.h cddmp_f.h cddtypes_f.h cdd.h cddmp.h cddtypes.h setoper.h cdd
```

Usage
-----

The contact planner is initiated with the script `humanoid_motion_planner.py` with the following options:

* `meta_path_generation_method`: Decide if each segment

```
python humanoid_motion_planner.py meta_path_generation_method hybrid traversability_select_criterion mean path_segmentation_generation_type motion_mode_and_traversability_segmentation environment_path environment_two_corridor surface_source random_surface_load_from_data start_env_id 0 end_env_id 0
```

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

* `contact_sequence_generation_method`: Decide how contact sequence is generated for each guiding path segment.<br/>
  `all_planning`: Use graph search planning in every segment.<br/>
  `all_retrival`: Retrive and adapt previously generated motion plan in evey segment.<br/>
  `hybrid`(Default): Use planning in segment with high traversability, and use retrival in segment with low traversability.

* `path_segmentation_type`: Decide how guiding path is segmented.<br/>
  `no_segmentation`: The planner will use every motion mode along the guiding path.<br/>
  `motion_mode_segmentation`: The planner will segment the guiding path at where motion mode changes.<br/>
  `motion_mode_and_traversability_segmentation`(Default): The planner will segment the guiding path at where motion mode changes, and then further decompose each segment based on the traversability.

* `traversability_threshold_type`: The type of threshold that the planner use to determine the contact sequence generation method.<br/>
  `mean`(Default): The planner uses the mean of the traversability of all torso transition in a segment to determine using planning or retrival method to generate contact sequence in the segment.<br/>
  `max`: The planner uses the max of the traversability of all torso transition in a segment to determine using planning or retrival method to generate contact sequence in the segment.

* `surface_source`: The source of the environment in planning. The repo provides 3 examples each for two-corridor and two-stair environment. The user can create new environment by adding new options in `update_environment` function in `environment_handler.py`.<br/>
  `two_corridor_environment`(Default): Randomly generate a two-corridor environment.<br/>
  `two_stair_environment`: Randomly generate a two-corridor environment.<br/>
  `load_from_data`: Load environment object file stored using `pickle` from path specified by `environment_path` parameter.

* `environment_path`: The folder which contains the stored environment object file.
* `start_env_id` and `end_env_id`: The first and last environment object file id loaded in the process.


Example Usage:
```
python humanoid_motion_planner.py contact_sequence_generation_method hybrid traversability_threshold_type mean path_segmentation_type motion_mode_and_traversability_segmentation environment_path environment_two_corridor surface_source load_from_data start_env_id 0 end_env_id 0
```
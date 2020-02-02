openrave_catkin
===============
openrave_catkin is a utility package that simplifies packaging of
[OpenRAVE](http://openrave.org/) models, plugins, and databases assets in a
Catkin workspace. This package defines a Catkin [environment
hook](http://docs.ros.org/fuerte/api/catkin/html/environment.html) that adds
these paths to your `OPENRAVE_DATA`, `OPENRAVE_DATABASES`, and
`OPENRAVE_PLUGINS` environment variables.

CMake Integration
-----------------
Additionally, this package defines the following CMake variables:

 - `OpenRAVE_DEVEL_DIR`: directory for OpenRAVE assets in the devel space
 - `OpenRAVE_INSTALL_DIR`: directory for OpenRAVE assets in the install space
 - `OpenRAVE_DATA_DIR`: subdirectory for models (e.g. `.kinbody.xml`, `.dae`)
 - `OpenRAVE_PLUGINS_DIR`: subdirectory for plugins
 - `OpenRAVE_DATABASES_DIR`: subdirectory for databases

Additionally, the variables exported by `find_package(OpenRAVE)` are available:

 - `OpenRAVE_INCLUDE_DIRS`
 - `OpenRAVE_COMPILE_FLAGS`
 - `OpenRAVE_LIBRARY_DIRS`
 - `OpenRAVE_LIBRARIES`
 - `OpenRAVE_LINK_FLAGS`

You can use these variables to install your models, plugins, and databases into
your OpenRAVE path. For example, you could install the model
`data/herb.kinbody.xml` from your Catkin package by adding these directives to
your `CMakeLists.txt` file:

    # TODO: Copy the model into the devel space.
    install(FILES "data/herb.kinbody.xml"
      DESTINATION "${OpenRAVE_INSTALL_DIR}/${OpenRAVE_DATA_DIR}"
    )

Building a plugin is more tedious because it requires specifying the correct
compilation flags, linker flags, and output options. We provide a CMake macro
`openrave_plugin` to simplify this:

Installation Paths
------------------
We define the following locations for OpenRAVE assets:

 - `OPENRAVE_DATA`: `share/openrave-MAJOR.MINOR/data`
 - `OPENRAVE_DATABASES`: `share/openrave-MAJOR.MINOR/databases`
 - `OPENRAVE_PLUGINS`: `share/openrave-MAJOR.MINOR/plugins`

In all cases, these paths are relative to the root of your of your current
Catkin devel or install space. This naming convention will was selected to
mirror the `${CMAKE_INSTALL_PREFIX}/share/openrave-MAJOR.MINOR` directory
used by OpenRAVE.

Multiple Versions of OpenRAVE
-----------------------------
This package does not support switching between OpenRAVE versions because this
use-case is not elegantly supported by Catkin. We would appreciate any thoughts
on the matter in the form of an issue. However, note that installation paths
used for OpenRAVE artifacts are namespaced by version number.

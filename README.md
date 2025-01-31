# Image Processing Template for using ROS2 and Python #

This repo serves as a simple template for working with ROS2 and doing non-real time image
processing.

# Set Up #

## Get the code ##

Since this is a template repo, it is not recommended to clone the code as the way we usually work
with Git and GitHub. The best way is to download the source code from

- The GitHub page. Go to https://github.com/strapsai/image-processing-template, choose the correct
  branch/tag and use the `Code` -> `Download ZIP` button to download the source code. 
- The GitHub releases. Go To https://github.com/strapsai/image-processing-template/releases, choose
  the best release of source code to download.

Extract the downloaded source code to your ROS2 workspace. DO NOT build your workspace now. Follow
the instructions in the next section.

## Prepare the ROS2 package ##

The extracted code works as a new ROS2 package in your workspace. Before the first build, we need to
modify the defualt parameters and settings.

Assuming the new name of the package is `image_analyzer`. Then the followings need to be renamed.

- Top level `image_processing_template` -> `image_analyzer`.
- `image_analyzer/image_processing_template` -> `image_analyzer/image_analyzer`.
- `image_analyzer/resource/image_processing_template` -> `image_analyzer/resource/image_analyzer`
- The `<name>` element and all related descriptions in `image_analyzer/package.xml`.
- The `<maintainer>` element in `image_analyzer/package.xml`
- All occurances of `image_processing_template` in the `setup.cfg` file.
- All occurances of `image_processing_template` in the `setup.py` file.
- All occurances of `image_processing_template` in the `image_analyzer/image_anlyzer/processor.py`
  file.

## Initial build ##

Go back to the workspace root folder and build the workspace. If the build succeeds, do

```bash
source ./install/setup.bash

# Use ros2 run to test the new package.
ros2 run image_analyzer processor
# Or, use ros2 launch to test the new package.
ros2 launch image_analyzer processor.launch.xml
```

Some prints will be shown in the termninal:

When using `ros2 run`, the example outputs look like

```
[INFO] [1707060083.198516210] [algorithm_node]: algorithm_node started.
```

When using `ros2 launch`, the example outputs look like

```
[INFO] [launch]: All log files can be found below /home/<user_home>/.ros/log/2024-02-04-10-22-27-348347-u2204mac-3940
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [processor-1]: process started with pid [3941]
[processor-1] [INFO] [1707060151.586663015] [algorithm_node]: algorithm_node started.
```

## Push to GitHub ##

The first thing after a successful initial build is setting up a new GitHub repo and push the
current ROS package to GitHub. Do this as usual. 

# Populate the Code #

The temlate is set up in such a way that the algorithm code should go into the
`image_analyzer/image_analyzer/processing_algorithm` folder. Do not put any algorithm code in
`image_analyzer/image_analyzer` foler although it looks like a normal Python package (with a
`__init__.py` in the folder). Otherwise, you will have nasty `import` errors.

In `image_analyzer/image_analyzer/processor.py` file, all `import` instructions are done assuming
the Python packages are properly installed system-wide and use absolute import such as `from
image_analyzer.processing_algorithm import ProcessingAlgorithm`.

# Who to Talk to? #

Yaoyu Hu.

(Please create GitHub issues for bug reports and feature requests.)

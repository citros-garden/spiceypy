---
slug: SpiceyPy Example with CITROS
title: SpiceyPy Example with CITROS
authors: [gtep]
tags: [CITROS]
---

Blog posts support [Docusaurus Markdown features](https://docusaurus.io/docs/markdown-features), such as [MDX](https://mdxjs.com/).

![jpg](img/img0.jpg "https://images.nasa.gov/details/PIA03883")

## Example overview 🌐 

This project is developed using ROS nodes and leverages the SpiceyPy library, a Python implementation of NASA's NAIF Spice toolkit. Its primary purpose is to provide the orbital trajectory information of the Cassini spacecraft relative to Saturn's barycenter within specified time intervals.

Users can input the desired time bounds, and the project utilizes SpiceyPy's powerful capabilities to retrieve accurate and precise orbital data for the Cassini spacecraft during the specified period.

You can find more information about SpiceyPy library on [SpiceyPy official website](https://spiceypy.readthedocs.io/en/v2.0.0/index.html).

## Code overview
This example consists of several files:
1. ```spiceypy_cassini.py```. This file is a ROS (Robot Operating System) node that utilizes the SpiceyPy library. It's objective is to launch the simulation and then to publish the simulation results via ROS topics. This code: initializes of a ROS node to manage the project, creates a publisher that sends trajectory data as a Float64MultiArray to the '/spiceypy_cassini/state' topic, uses user-defined inputs for the starting time, ending time, and the number of steps in the trajectory simulation, launches the simulation using SpiceyPy's capabilities, generating precise orbital data for Cassini during the specified time interval, and then publishes of trajectory data to the ROS topic, allowing users to access and analyze the spacecraft's orbital information.

2. ```util_spiceypy_cassini.py```. This Python code uses the SpiceyPy library to simulate the trajectory of NASA's Cassini spacecraft relative to Saturn's barycenter. It takes user-defined inputs for the simulation's start and end times and the number of time steps. The code then computes the spacecraft's positions at evenly spaced time intervals within the specified range. By leveraging SPICE kernels (as defined in 'cassMetaK.txt') and the 'spkpos' function, it calculates Cassini's positions in the J2000 reference frame. Once the simulation is complete, the code provides the collected position data for further analysis or visualization, making it a valuable tool for researchers and space enthusiasts interested in studying Cassini's mission trajectory.

You can find all the mathematical explanation in the [SpiceyPy example page](https://spiceypy.readthedocs.io/en/v2.0.0/exampleone.html). 

 The project also contains several additional files - kernels - to load Cassini trajectory data. They are stored under the ```resource``` folder: ```cassMetaK.txt``` is the main kernel file, where the other files are listed. For more on defining meta kernels in spice, please consult the [Kernel Required Reading](https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/kernel.html). 


## Local Usage 💻

All project installation and usage information also available in the project [GitHub page](https://github.com/citros-garden/spiceypy).

### Installation 🔨
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git clone git@github.com:citros-garden/spiceypy.git
```

### Build 🏠
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
2. Open ```/src/spiceypy_cassini/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
3. Build ROS 2 environment:
```bash 
colcon build
```
4. Source the environment:
```bash 
source install/local_setup.bash
```

### Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!


OR

:::tip

You can use prepared layout: Go to the ```Layout``` tab on the top panel, then click on import_layout button and select the file from foxglove_layouts folder.

:::

![gif](img/gif0.gif "FoxGlove example")

:::tip

The best way to process simulation results is CITROS notebook 🍋 :)

:::


### Run 🚀
1. Go back to the VS Code.
2. Launch ROS 2 package:
```bash 
ros2 launch spiceypy_cassini spiceypy_cassini.launch.py
```
3. Watch the FoxGlove plot built from results!

OR

:::tip

You can use Visual Code Tasks: simply press ```Alt+T``` and select the task you want to build. This will build, source and launch the project automaticly.

:::

![jpg](img/img1.jpg "FoxGlove example")

## CITROS usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS installation 🛫

First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/citros_cli).

### Configuring the project ⚙️
After all the prerequisites done, we can start configuring our project. The starting point is the Lunar_Starship devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
```bash 
citros init
```
Now you can see ```.citros``` folder in the explorer.

2. Configuring the setup. We need to set up the maximum perfomance available: timeout, CPU, GPU and Memory. To perform it, we need to define it in the ```.citros/simulations/simulation_spiceypy_cassini.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, 2 GPU and 1024 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find default setup in ```.citros/parameter_setups/default_param_setup.json```. The SpiceyPy simulation has the following ROS parameters:

$$
\begin{array}{|c|c|}
\hline
\text{Parameter} & \text{Description} \\
\hline
start\_t & \text{Initial date} \\
finish\_t & \text{Final date} \\
publish~freq & \text{frequency of publishing} \\
\hline
\end{array}
$$

:::note
The date format should be 'MMM DD, YYYY'.
:::

Don't forget to save the file!

### Syncing the project's setup 📡
Now we can sync our project settings with CITROS server:
```bash 
citros commit
citros push
```
### Running locally 🛋️
Since all the preparations done, we can launch it locally (your project should be built and sourced before that, check the instructions above):
```bash 
citros run -n 'spiceypy_cassini' -m 'local test run'
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

### Uploading Docker image to the CITROS database and running in the cloud 🛰️
1. We need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```

2. Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
citros run -n 'spiceypy_cassini' -m 'cloud test run' -r
```
Select the launch file (should be the only one here) by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and it will upload results to the CITROS database automaticly.

### CITROS Web usage and data analysis 🌌
#### Launching project via CITROS Web
The best way to use all the innovative capabilities of CITROS is through it's Web interface. The following manual explains how to run this project in the cloud and how to process the simualtion results.
The starting point is CITROS main page, user is logged in and the project Docker image is built and pushed to the cloud (see the [manual](#uploading-docker-image-to-the-citros-database-and-running-in-the-cloud-🛰️) above).
1. Go to the ```Repositories``` page clicking on the tab on the top;
2. Find your project and open it;
3. Navigate to the ```Runs``` tab;
4. Click on the ```Run Simulation``` button on the right;
5. Now you can choose the project and the simulation setup from the droplists, set the number of repeats and how many simulations should run in parallel, type the Name of the run and the additional message. This window also shows the perfomance preset.
6. We are ready to go! Start the Batch with the button below.

The simualtion launched! Open the Run you just started in the list on ```Runs``` page to check how is it going. In this page you can find all the runs of this batch. The number of runs here equals to the number of runs you've set before.
Navigate to the Run by clicking on it in the table:
* The main part of this page is a simulation's log. Here you can find all the logging information from all levels: from your code logs up to the CITROS system information.
* The right part of the page provides additional information about Events: the main stages of the simulation run.


#### Working with integrated Jupiter Notebooks
CITROS Web provides powerfull data analisys package, which is comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, the Jupiter Notebook support is built-in. 
Navigate to our project ```Code``` page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor's interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analisys package [here](https://citros.io/doc/docs_data_analysis).


## Extras
### FoxGlove examples

![png](img/Example0.png "FoxGlove example")
![png](img/Example1.png "FoxGlove example")

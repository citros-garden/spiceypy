![gif](/docs/img/main.gif "Maneuver example")
# General Information 🌍
This project contains one simple example using SpiceyPy (python vrsion of NASA NAIF Spice) lib for Python 3. `Spiceypy_cassini` returns cassini spacecraft orbital trajectory relative to the barycenter of Saturn between given time bounds. The input parameters are: ```start_t```,  startcan be set in the [params.yaml config file](/src/poliastro_simple_orbit/config/params.yaml). The result is an ephemerides of orbit (actually a part of it between given time bounds) with zero right ascension of the ascending node, argument of the pericenter an true anomaly for simplicity. All the calculations defined in [util_simple_orbit.py file](/src/poliastro_simple_orbit/poliastro_simple_orbit/util_simple_orbit.py).
Another important file is [poliastro_simple_orbit.py file](/src/poliastro_simple_orbit/poliastro_simple_orbit/poliastro_simple_orbit.py). ROS 2 node is used to call simulation function, get the results and publish it via ROS topic. You also can set publishing frequency (```publish_freq```) in the [params.yaml config file](/src/poliastro_simple_orbit/config/params.yaml) (more info about this file below).
2. `Poliastro_maneuver`. This package provides three orbits for the Hohmann transition: an initial orbit, an intermediate orbit, and a final orbit. Takes the radius of the initial orbit and the radius of the final orbit as input. You will get the ephemerides of these orbits, not the trajectory! 
3. `Poliastro_atmo_drag`. A simple example showing the effect of aerodynamic drag forces on an artificial satellite on low Earth orbit. Takes Earth diameter, drag coefficient, Keppler orbit parameters and maximum simulation time as inputs. The result is a plot of altitude by time and the flight time before hitting the surface.


You can find more information about orbital mechanics simulations with Poliastro on [Poliastro official website](https://poliastro-py.readthedocs.io/en/latest/user_guide.html)

# Installation 🛫
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git clone git@github.com:citros-garden/spiceypy.git
```

# Build 🛰
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
2. Open ```/src/<selected_example>/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
3. Build ROS2 environment:
```bash 
colcon build
```

# Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!

Last step is configuring the layout of FoxGlove. Navigate to the third tab on the left panel, and choose __*Plot*__ option. Each example has its unique ros2 publisher, so it's necessary to set Message paths in FoxGlove in advance. There are two ways to do it: using prepared layout file or manualy.

<details>
  <summary>To use prepared layout</summary>

1. Go to the fourth tab on the left panel, then click on three dots near *__Plot panel settings__* and choose *__Import/export settings...__*.

2. Copy code from file in [foxglove_layouts](/foxglove_layouts/) folder for the example you want to use.

3. Contragulations! You are ready to start!
</details>
<br>

OR 

<details>
  <summary>Manual plot layout configuration</summary>

1. For Poliastro_simple_orbit: '/poliastro_simple_orbit/state.data'. Set '/poliastro_simple_orbit/state.data[0]' as Message path in 'Series' tab and '/poliastro_simple_orbit/state.data[1]' as Message path in the 'X Axis' tab. This will provide a plot of orbit. 
2. For Poliastro_maneuver: '/Poliastro_maneuver/state.data'. Set '/poliastro_maneuver/state.data[0]' as Message path in 'Series' tab and '/Poliastro_maneuver/state.data[1]' as Message path in the 'X Axis' tab. This will provide full Hohmann maneuver including initial and final orbits.
3. For Poliastro_atmo_drag: '/Poliastro_atmo_drag/state.data'. Set '/Poliastro_atmo_drag/state.data[0]' as Message path in 'Series' tab and '/Poliastro_atmo_drag/state.data[1]' as Message path in the 'X Axis' tab. This will provide raw orbit data, and '/poliastro_atmo_drag/res.data' will provide total flight time before reaching surface. Although the best way to process simulation results is Citros notebook.
</details>
<br>


# Run 🚀
1. Go back to the VS Code.
2. Prepare your FoxGlove studio (previous step, if you haven't done it yet).
3. Launch selected ROS2 package:
```bash 
ros2 launch <selected_example> launch.py
```
4. Watch the FoxGlove plot built from results!

# Citros usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is Citros! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

## Citros integration
1. Build Docker image:
```bash
docker build -t spiceypy .
# OR *** when building from MAC M1 chip add FROM --platform=linux/amd64 ***
docker buildx build --platform linux/amd64 -t spiceypy .   
```
2. Login to Citros
 ```bash
citros login
citros docker-login
```
3. Tag your project and push it into Citros cloud
 ```bash
docker tag spiceypy us-central1-docker.pkg.dev/citros/lulav/spiceypy
docker push us-central1-docker.pkg.dev/citros/lulav/spiceypy
```


# Extras

![gif](/docs/img/main.gif "Maneuver example")
<br>
Simple maneuver example
<br>
<br>
![gif](/docs/img/simple_orbit.png "Simple orbit example")
Simple orbit example

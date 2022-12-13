# NTU IoT 2022 Course Final Project

## Files

This Git repository has git-submodules. Please initialize them if you
haven't done it yet.

```bash
git submodule init
git submodule update --recursive
```

The `src/` directory should look like this. Please explicitly check
each directory listed below must contain files and must not be empty.

```
src/
├── iot_agent/
├── carla_autoware_bridge_plus/
├── carla-ros-bridge/
│   └── carla_msgs/
└── astuff_sensor_msgs/
```

## Build

Build the Autoware repository first. Say it's at
`/path/to/autoware`. Remember to source the setup script. The step
must be done once whenever you start a new shell.

```bash
source /path/to/autoware/install/setup.bash
```

Go to this final project directory. Build the packages in the project.

```bash
make
```

The end of build message should look like this if it's successful.

```
Summary: 29 packages finished [7.94s]
cd judge && \
poetry install
Installing dependencies from lock file

No dependencies to install or update

Installing the current project: judge (0.1.0)
```

## Launch a Simulation

### Start Carla Simulator

Launch the Carla simulator first. To open simulation server on port
2005 on workstation for example:

```bash
/opt/carla-simulator/CarlaUE4.sh -carla-port=2005 -quality-level=Low
```

If the simulator crash occasionally, you can use a loop to restart
automatically.

```bash
while true; do
    /opt/carla-simulator/CarlaUE4.sh -carla-port=2005 -quality-level=Low
    sleep 1
done
```

### Run Simulation

If you're on SSH, remember to set `DISPLAY` to `:N`, where N is the
number provided by VNC server.

```bash
export DISPLAY=:N
```

Suppose that your Carla server port is 2005.  Launch the simulation
like this:

```bash
./launch.sh 2005
```

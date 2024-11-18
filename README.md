# Ground Control Station (PyQt5)

### Installation
```bash
cd && mkdir -p ground_control_station/src
cd ground_control_station/src
git clone https://github.com/furkansariyildiz/ground_control_station.git
colcon build --symlink-install --pacakges-select ground_control_station
```

### Running package
```bash
cd ground_control_station
source install/setup.bash
ros2 run ground_control_station ground_control_station
```

<p align="center">
  <img src="docs/images/ground-control-station.png" style="width: 100%; height: 50%"/>
</p>
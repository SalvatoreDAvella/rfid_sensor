# rfid_sensor

This is a Gazebo plugin for simulating RFID tags and RFID readers antennas. It has been tested on ROS Noetic and Gazebo 11.5.1. Once the tag is within the sensing range of the reader antenna, the antenna acquires phase and RSSI signals along with the name ID of the tag. The reader antenna(s) can be configured specifying the frequency, the sensing range, and some noise values that are used to account for multipath effects and uncertainties in the measurement of phase and RSSI signals.

## Usage
If you want to use it check this example [repo](https://github.com/SalvatoreDAvella/rfid_simulator_test)

## Citing

If you use this tool in a research project, please check the [paper](https://doi.org/10.1109/ACCESS.2022.3152199) and cite it as follows:
```
@article{davella2022RFID,
  author={D’Avella, Salvatore and Unetti, Matteo and Tripicchio, Paolo},
  journal={IEEE Access}, 
  title={RFID Gazebo-Based Simulator With RSSI and Phase Signals for UHF Tags Localization and Tracking}, 
  year={2022},
  volume={10},
  number={},
  pages={22150-22160},
  publisher={IEEE},
  doi={10.1109/ACCESS.2022.3152199},
  issn={2169-3536}
}
```

## License
Distributed under the GNU License. See LICENSE for more information.

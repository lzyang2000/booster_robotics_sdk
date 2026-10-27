# booster_lowlevel

A small Python binding over this SDK for low-level control of one Booster robot (K1, T1
or T2), in the spirit of `unitree_sdk2_wrapper`'s `unitree_interface`. One file,
`booster_lowlevel.cpp`, linked against the prebuilt `lib/<arch>/libbooster_robotics_sdk.a`.

```bash
PYTHON=/path/to/python ./build.sh   # needs cmake, g++ and pybind11 in that Python
# -> build/booster_lowlevel.cpython-*.so: copy it onto that Python's path
```

`BoosterInterface(network_interface, num_motors, domain_id=0)` (`create_t2(net)`: 31 joints):

| Method | Does |
|---|---|
| `read_low_state()` | the latest `rt/low_state`: `imu.rpy`, `imu.gyro`, `imu.acc`, `motor.q/dq/tau_est/temperature/lost` (serial joint order), `age` (s since it arrived, inf before the first), `count` |
| `read_remote_controller()` | the latest `rt/remote_controller_state` (`a`, `b`, `x`, `y`, `lb`, `rb`, `lt`, `rt`, `ls`, `rs`, `back`, `start`, `hat_*`, sticks, `age`); a button down in any message since the last read reads as down |
| `create_zero_command()`, `write_low_command(cmd)` | one SERIAL `LowCmd` on `rt/joint_ctrl` per call (`q_target`, `dq_target`, `kp`, `kd`, `tau_ff`; `weight` and `mode` 0, as Booster's deploy sends) |
| `change_mode(RobotMode.X)`, `get_mode()` | the locomotion RPC (`rt/LocoApiTopic`): the status (0 = success, 100 = no answer) / the mode as an int (-1 if the call failed, after 5 s). `rt/joint_ctrl` acts only in `CUSTOM` |
| `low_state_publishers()`, `command_subscribers()` | matched DDS endpoints |

`FakeRobot(network_interface, num_motors, domain_id=0, mode=RobotMode.PREPARE)` is the
robot's side of the same topics and RPC, for desk tests: `publish_low_state(rpy, gyro,
acc, q, dq)`, `publish_remote([button names])`, `read_command()` (the latest command or
None, and the count), `mode()` / `set_mode()` (ints; ChangeMode and GetMode answer from it).
It also talks to a `BoosterInterface` in the same process. A process has one DDS
participant (the SDK's `ChannelFactory`), which keeps the network interface and domain of
the first object created in it.

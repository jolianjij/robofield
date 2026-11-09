# API reference

The Python API of RoboField, generated from the code's docstrings. Start with the
[architecture](../architecture.md) page for how the pieces fit together.

| Package | What it holds |
|---|---|
| [robofield.core](core.md) | Shared types: poses, robot and world state, commands, the backend interface, the config loader |
| [robofield.control](control.md) | The car model (kinematics, motor curve), the EKF, the delay predictor, the controllers |
| [robofield.app](app.md) | The Control Center, the safety layer, policy driving, the app runtime, jobs, the camera hub |
| [robofield.lab](lab.md) | Writing your own policies, systems and tasks |
| [robofield.games and envs](games.md) | Game plugins and the Gymnasium / PettingZoo environments |
| [robofield.sim and real](backends.md) | The MuJoCo simulator and the real field, behind one interface |
| [robofield.comm and vision](comm_vision.md) | Radio packets, the UDP links, ArUco pose estimation |

Programs written in the app's Lab only need [robofield.lab](lab.md).

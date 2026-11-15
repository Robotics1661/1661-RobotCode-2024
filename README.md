# 1661-RobotCode-2024

Java robot code for the 2024 season (CRESCENDO) from [Griffitrons 1661](https://team1661.com), the FIRST Robotics Competition team at The Buckley School.

## Features

- **Swerve drivetrain** using CTRE Phoenix 6 with Tuner X generated constants
- **Shooter, intake, four-bar and climber** subsystems with matching commands, including an assisted far shot
- **Vision** subsystem and Limelight helpers
- **PathPlanner** autonomous routines built from named command pieces (`commands/named`, `commands/autos`),
  with an autonomous mode selector
- SysId commands for characterizing the four-bar
- `annotation-processor/`: a Gradle module used at compile time (annotation processing) by the robot project

Vendor libraries: Phoenix 5 and 6, PathPlanner, REVLib and the SDS swerve library (see `vendordeps/`).

## Building and running

You'll need [WPILib 2024](https://docs.wpilib.org/) (it bundles the JDK and Gradle). Then, from the repo root:

```bash
./gradlew build          # compile and run checks
./gradlew simulateJava   # run in the WPILib simulator
./gradlew deploy         # deploy to a roboRIO (must be connected to the robot)
```

On Windows use `gradlew.bat` instead of `./gradlew`. The team number (1661) lives in
`.wpilib/wpilib_preferences.json`.

## License

Team-written code is released under the [MIT License](LICENSE). Files generated from the WPILib
project template keep their original headers, and WPILib itself is covered by the BSD-style license in
[WPILib-License.md](WPILib-License.md). Vendor libraries in `vendordeps/` are covered by their own licenses.

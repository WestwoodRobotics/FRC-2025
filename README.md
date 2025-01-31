# 2025KiwiBeta

## Project Description
This repository contains code for controlling various robot subsystems, including swerve drive, shooter, intake, and more. The code is written in Java and is designed to be used with the WPILib library for FRC (FIRST Robotics Competition) robots.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgments](#acknowledgments)

## Installation
To install and set up the project, follow these steps:

1. Clone the repository:
   ```sh
   git clone https://github.com/WestwoodRobotics/2025KiwiBeta.git
   cd 2025KiwiBeta
   ```

2. Install the required dependencies:
   - Ensure you have Java Development Kit (JDK) 17 installed.
   - Install WPILib and other necessary libraries using the WPILib installer.

3. Open the project in your preferred IDE (e.g., Visual Studio Code, IntelliJ IDEA).

## Usage
To use the project, follow these steps:

1. Build the project:
   ```sh
   ./gradlew build
   ```

2. Deploy the code to the robot:
   ```sh
   ./gradlew deploy
   ```

3. Run the robot code:
   - Connect to the robot using the FRC Driver Station.
   - Enable the robot and select the desired operation mode.

## Configuration
The project includes various configuration options that can be set using environment variables or configuration files. Some key configuration files include:

- `src/main/java/frc/robot/Constants.java`: Contains robot-wide numerical or boolean constants.
- `src/main/java/frc/robot/Robot.java`: Main robot class that extends `TimedRobot`.
- `src/main/java/frc/robot/RobotContainer.java`: Contains subsystems, OI devices, and commands.

## Project Structure
The project is organized into the following directories and files:

- `src/main/java/frc/robot/commands/axe/AxePIDCommand.java`: Command for controlling the axe subsystem using PID.
- `src/main/java/frc/robot/commands/preRoller/preRollerSenseCommand.java`: Command for controlling the pre-roller subsystem.
- `src/main/java/frc/robot/commands/shooter/shooterPIDCommand.java`: Command for controlling the shooter subsystem using PID.
- `src/main/java/frc/robot/sensors/NeoADIS16470.java`: Utility class for handling the NeoADIS16470 sensor.
- `src/main/java/frc/robot/subsystems/utils/LimelightHelpers.java`: Utility class for handling the Limelight camera.
- `src/main/java/frc/robot/subsystems/swerve/SwerveDrive.java`: Represents the swerve drive system.
- `src/main/java/frc/robot/subsystems/swerve/MAXSwerveModule.java`: Represents the swerve drive module.

## Contributing
We welcome contributions to the project! To contribute, follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix:
   ```sh
   git checkout -b feature/your-feature-name
   ```
3. Make your changes and commit them:
   ```sh
   git commit -m "Add your commit message"
   ```
4. Push your changes to your forked repository:
   ```sh
   git push origin feature/your-feature-name
   ```
5. Create a pull request to the main repository.

## License
This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for more information.


## Acknowledgments
We would like to thank the following third-party libraries, tools, and resources used in this project:

- WPILib
- PathPlanner
- REVLib

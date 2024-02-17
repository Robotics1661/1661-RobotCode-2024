// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.SWERVE_MAX_ANGULAR_RATE;
import static frc.robot.Constants.SWERVE_MAX_SPEED;

import java.io.File;
import java.lang.reflect.Constructor;
import java.lang.reflect.Executable;
import java.lang.reflect.Method;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TestFourBarCommand;
import frc.robot.commands.named.INamedCommandBuilder;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.commands.sysid.FourBarSysIdCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ConfigurationBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = TunerConstants.DriveTrain;
  private final FourBarSubsystem m_fourBarSubsystem = new FourBarSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // Controllers
  private final JoyXboxWrapper m_combined_controller = new JoyXboxWrapper(0, 3, true);

  // Swerve-specific control
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      //.withDeadband(SWERVE_MAX_SPEED * 0.1).withRotationalDeadband(SWERVE_MAX_ANGULAR_RATE * 0.1) // Do NOT Add a 10% deadband (already deadbanded)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(SWERVE_MAX_SPEED);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(m_drivetrainSubsystem.applyRequest(() -> drive
      .withVelocityX(-modifyAxis(m_combined_controller.getLateralY(), "left_y", true) * SWERVE_MAX_SPEED) // Drive forward
      .withVelocityY(-modifyAxis(m_combined_controller.getLateralX(), "left_x", true) * SWERVE_MAX_SPEED) // Drive left
      .withRotationalRate(-modifyAxis(m_combined_controller.getRotation(), "right_x", true) * SWERVE_MAX_ANGULAR_RATE) // Spin
    ));
    new Trigger(m_combined_controller::getBrakeButton).whileTrue(m_drivetrainSubsystem.applyRequest(() -> brake));
    new Trigger(m_combined_controller::getZeroButton).onTrue(m_drivetrainSubsystem.runOnce(() -> m_drivetrainSubsystem.seedFieldRelative()));

    m_fourBarSubsystem.setDefaultCommand(new TestFourBarCommand(
      m_fourBarSubsystem,
      () -> modifyAxis(m_combined_controller.getFourBarSpeed(), "four_bar_speed", true)
    ));
    m_intakeSubsystem.setDefaultCommand(new IntakeCommand(
      m_intakeSubsystem,
      m_combined_controller::getIntake
    ));

    if (Utils.isSimulation()) {
      m_drivetrainSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_drivetrainSubsystem.registerTelemetry(logger::telemeterize);

    registerNamedCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Constants.AUTONOMOUS_MODE.getCommand(m_drivetrainSubsystem);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value, String label, boolean square) {
    SmartDashboard.putNumber("controller_"+label+"_raw", value);
    // Deadband
    value = deadband(value, m_combined_controller.isSoftDisabled() ? 0.2 : 0.1); // was 0.05
    SmartDashboard.putNumber("controller_"+label+"_db_cor", value);

    if (square) {
      // Square the axis
      value = Math.copySign(value * value, value);

      SmartDashboard.putNumber("controller_"+label+"_sq", value);
    }

    return value;
  }

  private final SysIdMode SYSID_MODE = SysIdMode.SWERVE_STEERING;

  public Command getSysIdCommand() {
    return switch (SYSID_MODE) {
      case FOUR_BAR -> FourBarSysIdCommand.create(m_fourBarSubsystem, m_combined_controller::getSysIdSafety);
      case SWERVE_STEERING -> m_drivetrainSubsystem.getSteeringSysIdCommand(m_combined_controller::getSysIdSafety);
    };
  }

  private enum SysIdMode {
    FOUR_BAR,
    SWERVE_STEERING
  }

  private void registerNamedCommands() {
    System.out.println("\n\n\nRegistering named commands");

    Reflections reflections = new Reflections(new ConfigurationBuilder()
      .forPackages("frc.robot.commands.named")
      .addScanners(Scanners.MethodsAnnotated, Scanners.ConstructorsAnnotated)
    );

    Set<Method> annotatedMethods = reflections.getMethodsAnnotatedWith(INamedCommand.class);

    @SuppressWarnings({"rawtypes", "unchecked"})
    Set<Constructor<?>> annotatedConstructors = (Set<Constructor<?>>) (Set) reflections.getConstructorsAnnotatedWith(INamedCommand.class);

    Set<Executable> combined = new HashSet<>(annotatedMethods.size() + annotatedConstructors.size());
    combined.addAll(annotatedMethods);
    combined.addAll(annotatedConstructors);

    for (Executable executable : combined) {
      String name = executable.getAnnotation(INamedCommand.class).value();
      Optional<INamedCommandBuilder> builder = INamedCommandBuilder.match(executable);
      if (builder.isPresent()) {
        NamedCommands.registerCommand(name, builder.get().build(m_drivetrainSubsystem, m_fourBarSubsystem, m_intakeSubsystem));
        System.out.println("Registered named command %s".formatted(name));
      } else {
        String msg = "ERROR: Method %s annotated as a named command but doesn't implement the Builder interface".formatted(executable.getName());
        System.out.println(msg);
        if (Utils.isSimulation()) {
          throw new RuntimeException(msg);
        }
      }
    }

    // Register special `pathfind_to_"..."` commands
    Stream.of(new File(Filesystem.getDeployDirectory(), "pathplanner/paths/").listFiles())
      .filter(file -> !file.isDirectory())
      .map(File::getName)
      .filter(name -> name.endsWith(".path"))
      .map(name -> name.substring(0, name.length() - 5))
      .forEach(pathName -> {
        String commandName = "pathfind_to_\""+pathName+"\"";
        NamedCommands.registerCommand(commandName, m_drivetrainSubsystem.pathfindTo(PathPlannerPath.fromPathFile(pathName)));
        System.out.println("Registered pathfinding command %s".formatted(commandName));
      });

    System.out.println("Registered named commands\n\n\n");
  }
}

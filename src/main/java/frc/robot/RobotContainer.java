// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DISABLE_CLIMBER;
import static frc.robot.Constants.SWERVE_MAX_ANGULAR_RATE;
import static frc.robot.Constants.SWERVE_MAX_SPEED;
import static frc.robot.util.AutonomousDebugger.markAutoStart;
import static frc.robot.util.AutonomousDebugger.printAutoEnd;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.InvocationTargetException;
import java.util.stream.Stream;

import javax.annotation.Nullable;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.FourBarCommand;
import frc.robot.commands.InitFourBarCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand;
import frc.robot.commands.sysid.FourBarSysIdCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutonomousInput;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.DoubleRingBuffer;

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
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = DISABLE_CLIMBER ? null : new ClimberSubsystem();

  private final AutonomousInput m_autonomousInput = new AutonomousInput(
    m_drivetrainSubsystem,
    m_intakeSubsystem,
    m_fourBarSubsystem,
    m_shooterSubsystem
  );

  // Controllers
  private final JoyXboxWrapper m_combined_controller = new JoyXboxWrapper(0, 3, true);

  // average control out over `SMOOTH_PERIOD` inputs
  private static final int SMOOTH_PERIOD = 7;
  private final DoubleRingBuffer m_drive_forward_buffer = new DoubleRingBuffer(SMOOTH_PERIOD);
  private final DoubleRingBuffer m_drive_sideways_buffer = new DoubleRingBuffer(SMOOTH_PERIOD);

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
      .withVelocityX(m_drive_forward_buffer.smooth(-modifyAxis(m_combined_controller.getLateralY(), "left_y", true) * SWERVE_MAX_SPEED)) // Drive forward
      .withVelocityY(m_drive_sideways_buffer.smooth(-modifyAxis(m_combined_controller.getLateralX(), "left_x", true) * SWERVE_MAX_SPEED)) // Drive left
      .withRotationalRate(-modifyAxis(m_combined_controller.getRotation(), "right_x", true) * SWERVE_MAX_ANGULAR_RATE) // Spin
    ));
    new Trigger(m_combined_controller::getBrakeButton).whileTrue(m_drivetrainSubsystem.applyRequest(() -> brake));
    new Trigger(m_combined_controller::getZeroButton).onTrue(m_drivetrainSubsystem.runOnce(() -> m_drivetrainSubsystem.seedFieldRelative()));
    new Trigger(m_combined_controller::getZeroButton180).onTrue(m_drivetrainSubsystem.runOnce(() -> m_drivetrainSubsystem.seedFieldRelative180()));
    
    new Trigger(m_combined_controller::getPoseFromLLButton).onTrue(m_drivetrainSubsystem.runOnce(() -> m_drivetrainSubsystem.useLimelightPose()));
    new Trigger(m_combined_controller::getDriveToAmpButton).whileTrue(new PathPlannerAuto("Manual Amp Shot"));

    //*
    m_fourBarSubsystem.setDefaultCommand(new FourBarCommand(
      m_fourBarSubsystem,
      () -> modifyAxis(m_combined_controller.getFourBarSpeed(), "four_bar_speed", true),
      m_combined_controller::getFourBarOrigin,
      m_combined_controller::getFourBarIntake,
      m_combined_controller::getFourBarAmp,
      m_combined_controller::getFourBarSpeaker,
      m_combined_controller::getFourBarFarSpeaker,
      m_combined_controller::getFourBarOffsetIncrease,
      m_combined_controller::getFourBarOffsetDecrease
    )); // */

    new Trigger(m_combined_controller::getFourBarInitialize)
      .whileTrue(new InitFourBarCommand(m_fourBarSubsystem));


    m_intakeSubsystem.setDefaultCommand(new IntakeCommand(
      m_intakeSubsystem,
      m_combined_controller::getIntake,
      () -> modifyAxis(m_combined_controller.getIntakeReverse(), "intake reverse", false)
    ));

    m_shooterSubsystem.setDefaultCommand(new ShooterCommand(
      m_shooterSubsystem,
      m_combined_controller::getAmpShot,
      m_combined_controller::getSpeakerShot,
      //m_combined_controller::getTestShooterSpeedIncrease,
      //m_combined_controller::getTestShooterSpeedDecrease,
      TimedIntakeCommand.makeScheduler(m_intakeSubsystem)
    ));

    if (!DISABLE_CLIMBER) {
      m_climberSubsystem.setDefaultCommand(new ClimberCommand(
        m_climberSubsystem,
        m_combined_controller::getClimberExtend,
        m_combined_controller::getClimberRetract
      ));
    }

    if (Utils.isSimulation()) {
      m_drivetrainSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_drivetrainSubsystem.registerAdditionalTelemetry(logger::telemeterize);

    registerNamedCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("\n\n\n\nCreated Autonomous Command\n\n\n\n");
    return new SequentialCommandGroup(
      markAutoStart(),
      new InitFourBarCommand(m_fourBarSubsystem),
      Constants.AUTONOMOUS_MODE.getCommand(m_autonomousInput),
      printAutoEnd()
    );
  }

  public void stopAllOnDisable() {
    m_autonomousInput.stopAll();
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

  private final SysIdMode SYSID_MODE = SysIdMode.FOUR_BAR;

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

  /**
   * 
   * @param parameterNodes contents of <parameters/> tag
   * @param parameterArray String[1]
   */
  private static void readAutoNamedCommandParameters(NodeList parameterNodes, String[] parameterArray) {
    int idx = 0;
    for (int i = 0; i < parameterNodes.getLength(); i++) {
      Node child = parameterNodes.item(i);
      if (child.getNodeName().equals("parameter")) {
        parameterArray[idx] = child.getTextContent();
        idx++;
      }
    }
  }

  private void registerNamedCommands() {
    System.out.println("\n\n\nRegistering named commands");

    InputStream namedCommandsStream = getClass().getResourceAsStream("/resources/frc/robot/named_commands.xml");
    DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance();
    Document doc = null;
    try {
      DocumentBuilder docBuilder = docBuilderFactory.newDocumentBuilder();
      doc = docBuilder.parse(namedCommandsStream);
    } catch (ParserConfigurationException | SAXException | IOException e) {
      System.err.println("Failed to load generated named commands file");
      e.printStackTrace();
    }
    if (doc != null) {
      Node namedCommands = doc.getFirstChild();

      NodeList children = namedCommands.getChildNodes();
      Outer: for (int i = 0; i < children.getLength(); i++) {
        Node command = children.item(i);
        if (!command.getNodeName().equals("command")) continue;

        String commandName = command.getAttributes().getNamedItem("name").getTextContent();
        System.out.println("Beginning registration for command named: "+commandName);

        NodeList commandChildren = command.getChildNodes();

        @Nullable String className = null;
        @Nullable String methodName = null;
        @Nullable String returnType = null;
        boolean foundConstructor = false;
        String[] parameters = new String[]{null};

        for (int j = 0; j < commandChildren.getLength(); j++) {
          Node inner = commandChildren.item(j);
          switch (inner.getNodeName()) {
            case "class" -> {
              className = inner.getTextContent();
            }
            case "method" -> {
              NodeList methodChildren = inner.getChildNodes();
              for (int k = 0; k < methodChildren.getLength(); k++) {
                Node methodInner = methodChildren.item(k);
                switch (methodInner.getNodeName()) {
                  case "name" -> methodName = methodInner.getTextContent();
                  case "return" -> returnType = methodInner.getTextContent();
                  case "parameters" -> readAutoNamedCommandParameters(methodInner.getChildNodes(), parameters);
                };
              }
            }
            case "constructor" -> {
              readAutoNamedCommandParameters(inner.getChildNodes(), parameters);
              foundConstructor = true;
            }
          }
        }

        final ClassLoader classLoader = RobotContainer.class.getClassLoader();

        // Get classes for parameters
        Class<?>[] paramClasses = new Class[parameters.length];
        for (int j = 0; j < parameters.length; j++) {
          try {
            paramClasses[j] = classLoader.loadClass(parameters[j]);
          } catch (ClassNotFoundException e) {
            System.err.println("Failed to register named command '"+commandName+"'");
            e.printStackTrace();
            if (Utils.isSimulation()) {
              throw new RuntimeException("Failed to register named command '"+commandName+"'");
            }
            continue Outer;
          }
        }

        try {
          if (foundConstructor) {
            Class<?> clazz = classLoader.loadClass(className);
            var constructor = clazz.getConstructor(paramClasses);
            constructor.setAccessible(true);
            NamedCommands.registerCommand(commandName, (Command) constructor.newInstance(m_autonomousInput));
            System.out.println("Registered named command %s".formatted(commandName));
          } else if (methodName != null && returnType != null) {
            Class<?> clazz = classLoader.loadClass(className);
            var method = clazz.getMethod(methodName, paramClasses);
            method.setAccessible(true);
            NamedCommands.registerCommand(commandName, (Command) method.invoke(null, m_autonomousInput));
            System.out.println("Registered named command %s".formatted(commandName));
          } else {
            System.err.println("Failed to register named command '"+commandName+"', no constructor or method found");
            if (Utils.isSimulation()) {
              throw new RuntimeException("Failed to register named command '"+commandName+"'");
            }
            continue Outer;
          }
        }  catch (ClassNotFoundException | NoSuchMethodException | SecurityException | InstantiationException
                  | IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          System.err.println("Failed to register named command '"+commandName+"'");
          e.printStackTrace();
          if (Utils.isSimulation()) {
            throw new RuntimeException("Failed to register named command '"+commandName+"'");
          }
          continue Outer;
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

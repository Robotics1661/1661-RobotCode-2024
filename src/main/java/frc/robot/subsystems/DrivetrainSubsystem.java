package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.ENABLE_LL_VISION_ESTIMATE;
import static frc.robot.Constants.PATH_FOLLOW_CONFIG;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.VisionSubsystem.PoseLatency;
import frc.robot.util.MathUtil;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final VisionSubsystem m_visionSystem = new VisionSubsystem();
    private boolean useLimelightPose = false;
    private final Field2d m_field;

    private static boolean mirrorAlliancePath() {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public DrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        AutoBuilder.configureHolonomic(
            m_odometry::getEstimatedPosition,
            this::seedFieldRelative,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            PATH_FOLLOW_CONFIG,
            DrivetrainSubsystem::mirrorAlliancePath,
            this // Reference to this subsystem to set requirements
        );

        m_field = new Field2d();

        SmartDashboard.putData("Field", m_field);

        // Hacky way to schedule code into the odometry thread
        registerTelemetry((state) -> odometryUpdate());
    }
    public DrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 0, modules);
    }

    // In m/s/s
    private double getTotalAcceleration_mss() {
        final Pigeon2 pigeon = getPigeon2();

        double x = pigeon.getAccelerationX().refresh().getValueAsDouble() - pigeon.getGravityVectorX().refresh().getValueAsDouble();
        double y = pigeon.getAccelerationY().refresh().getValueAsDouble() - pigeon.getGravityVectorY().refresh().getValueAsDouble();
        double z = pigeon.getAccelerationZ().refresh().getValueAsDouble() - pigeon.getGravityVectorZ().refresh().getValueAsDouble();

        return Math.sqrt(x*x + y*y + z*z);
    }

    private double getTotalCurrentDraw() {
        double total = 0;

        for (int i = 0; i < 4; i++) {
            var module = getModule(i);

            total += Math.abs(module.getDriveMotor().getSupplyCurrent().refresh().getValueAsDouble());
            total += Math.abs(module.getSteerMotor().getSupplyCurrent().refresh().getValueAsDouble());
        }

        return total;
    }

    private void odometryUpdate() {
        //System.out.println("Odometry update");
        if (ENABLE_LL_VISION_ESTIMATE && !Utils.isSimulation()) {
            updatePoseEstimatorWithVisionBotPose();
        }
    }

    @Override
    public void periodic() {
        final Pigeon2 pigeon = getPigeon2();

        SmartDashboard.putNumber("gyro", pigeon.getAngle());
        SmartDashboard.putNumber("acceleration", getTotalAcceleration_mss());
        
        if (!Utils.isSimulation()) {
            SmartDashboard.putNumber("driveDraw", getTotalCurrentDraw());
        }
        m_field.setRobotPose(m_odometry.getEstimatedPosition());
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void sysIdSteeringVelocityDrive(Measure<Voltage> voltage) {
        getModule(0).getSteerMotor().setControl(new VoltageOut(voltage.baseUnitMagnitude()));
    }

    private void sysIdSteeringLogger(SysIdRoutineLog log) {
        var logger = log.motor("right");
        var motor = getModule(0).getSteerMotor();
        logger.voltage(Units.Volts.of(motor.getMotorVoltage().refresh().getValueAsDouble()));
        logger.angularVelocity(Units.RotationsPerSecond.of(motor.getVelocity().getValueAsDouble()));
        logger.angularPosition(Units.Rotations.of(motor.getPosition().getValueAsDouble()));
        //rightLogger.angularPosition(Units.Meters.ofBaseUnits(m_motorRight.getPosition().refresh().getValueAsDouble()));
    }

    private SysIdRoutine createSteeringSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::sysIdSteeringVelocityDrive, this::sysIdSteeringLogger, this)
        );
    }

    public Command getSteeringSysIdCommand(BooleanSupplier safetySupplier) {
        return new SequentialCommandGroup(
            createSteeringSysIdRoutine().quasistatic(Direction.kForward),
            createSteeringSysIdRoutine().quasistatic(Direction.kReverse),
            createSteeringSysIdRoutine().dynamic(Direction.kForward),
            createSteeringSysIdRoutine().dynamic(Direction.kReverse)
        ).onlyWhile(safetySupplier);
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    }

    private final SwerveRequest.ApplyChassisSpeeds chassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        this.setControl(chassisSpeedsRequest.withSpeeds(chassisSpeeds));
    }

    public Command pathfindTo(PathPlannerPath targetPath) {
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            MathUtil.degreesToRadians(540), MathUtil.degreesToRadians(720));

        return new PathfindHolonomic(
            targetPath,
            constraints,
            m_odometry::getEstimatedPosition,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            PATH_FOLLOW_CONFIG,
            //3.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
            DrivetrainSubsystem::mirrorAlliancePath,
            this // Reference to this subsystem to set requirements
        );
    }

    public Command pathfindTo(Pose2d target) {
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            MathUtil.degreesToRadians(540), MathUtil.degreesToRadians(720)
        );

        if (mirrorAlliancePath()) {
            target = GeometryUtil.flipFieldPose(target);
        }

        return new PathfindHolonomic(
            target,
            constraints,
            0.0,
            m_odometry::getEstimatedPosition,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            PATH_FOLLOW_CONFIG,
            this
        );
    }

    public void updatePoseEstimatorWithVisionBotPose() {
        PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
        // invalid LL data
        if (visionBotPose.pose2d().getX() == 0.0) {
            //System.out.println("[LL] Invalid LL data");
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = m_odometry.getEstimatedPosition().getTranslation()
            .getDistance(visionBotPose.pose2d().getTranslation());

        if (m_visionSystem.areAnyTargetsValid()) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
                xyStds = 0.5;
                degStds = 6;
                //System.out.println("[LL] >= 2 targets");
            }
            // 1 target with large area and close to estimated pose
            else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
                //System.out.println("[LL] 1 big target");
            }
            // 1 target farther away and estimated pose is close
            else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
                //System.out.println("[LL] 1 small target");
            }
            else if (useLimelightPose) {
                useLimelightPose = false;
                xyStds = 0;
                degStds = 0;
                System.out.println("[LL] Using limelight pose anyway");
            }
            // conditions don't match to add a vision measurement
            else {
                //System.out.println("[LL] Too large pose difference: "+poseDifference);
                return;
            }

            m_odometry.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, degreesToRadians(degStds)));
            m_odometry.addVisionMeasurement(visionBotPose.pose2d(),
                visionBotPose.timestampSeconds());
        }
    }

    /**
     * Takes the current orientation of the robot and makes it X backward for
     * field-relative
     * maneuvers.
     */
    public void seedFieldRelative180() {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = getState().Pose.getRotation()
                .plus(Rotation2d.fromDegrees(180));
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Register the specified lambda to be executed whenever our SwerveDriveState function
     * is updated in our odometry thread.
     * <p>
     * It is imperative that this function is cheap, as it will be executed along with
     * the odometry call, and if this takes a long time, it may negatively impact
     * the odometry of this stack.
     * <p>
     * This can also be used for logging data if the function performs logging instead of telemetry
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerAdditionalTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        try {
            m_stateLock.writeLock().lock();
            if (m_telemetryFunction == null) {
                m_telemetryFunction = telemetryFunction;
            } else {
                Consumer<SwerveDriveState> prev = m_telemetryFunction;
                m_telemetryFunction = (state) -> {
                    prev.accept(state);
                    telemetryFunction.accept(state);
                };
            }
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    public void useLimelightPose() {
        try {
            m_stateLock.writeLock().lock();
            useLimelightPose = true;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }
}

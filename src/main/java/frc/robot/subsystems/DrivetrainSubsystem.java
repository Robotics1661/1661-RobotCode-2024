package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public DrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public DrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // In m/s/s
    private double getTotalAcceleration_mss() {
        final Pigeon2 pigeon = getPigeon2();

        double x = pigeon.getAccelerationX().refresh().getValueAsDouble();
        double y = pigeon.getAccelerationY().refresh().getValueAsDouble();
        double z = pigeon.getAccelerationZ().refresh().getValueAsDouble();

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

    @Override
    public void periodic() {
        final Pigeon2 pigeon = getPigeon2();

        SmartDashboard.putNumber("gyro", pigeon.getAngle());
        SmartDashboard.putNumber("acceleration", getTotalAcceleration_mss());
        SmartDashboard.putNumber("driveDraw", getTotalCurrentDraw());
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
}

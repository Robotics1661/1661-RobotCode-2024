package frc.robot.commands.autos.pieces;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import static frc.robot.Constants.SWERVE_MAX_SPEED;
import static frc.robot.util.AutonomousDebugger.autoDbg;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardCommand extends Command {
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double m_durationSeconds;

    private double startTime;

    public DriveForwardCommand(DrivetrainSubsystem drivetrainSubsystem, double durationSeconds) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_durationSeconds = durationSeconds;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        startTime = Timer.getFPGATimestamp();
        autoDbg("Starting drive forward");
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > (startTime + m_durationSeconds);
    }

    @Override
    public void execute() {
        super.execute();

        if (!Utils.isSimulation()) {
            m_drivetrainSubsystem.setControl(m_drive
                .withVelocityX(0.15 * SWERVE_MAX_SPEED)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        m_drivetrainSubsystem.setControl(m_brake);
        autoDbg("Drive forward complete");
    }
}

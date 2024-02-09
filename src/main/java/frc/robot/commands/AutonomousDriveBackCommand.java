package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousDriveBackCommand extends Command {
    private double m_startTime;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public AutonomousDriveBackCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_startTime = Timer.getFPGATimestamp(); //just as a failsafe
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(this.m_drivetrainSubsystem);
    }

    /*
     * Get time this command has been running, in seconds
     */
    private double getRunningTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_drivetrainSubsystem.seedFieldRelative();
    }

    private boolean shouldMove() {
        return getRunningTime() < 3.5;
    }

    @Override
    public void execute() {
        if (!shouldMove()) return; //failsafe, should finalize
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.applyRequest(() -> drive
           .withVelocityX(0.8)
        );
    }

    @Override
    public boolean isFinished() {
        return !shouldMove();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.applyRequest(() -> brake);
    }
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousDriveBackAndForthCommand extends Command {
    private double m_startTime;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public AutonomousDriveBackAndForthCommand(DrivetrainSubsystem drivetrainSubsystem) {
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
        m_drivetrainSubsystem.calibrateGyro();
    }

    private boolean shouldMove() {
        return getRunningTime() < 9.5;
    }

    @Override
    public void execute() {
        if (!shouldMove()) return; //failsafe, should finalize
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if (getRunningTime() < 3.5) { //Drive to balance
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            .8,
                            0,
                            0,
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
                    
            );
        } else if (getRunningTime() < 5.75) { //Drive forward and off (for another 2 seconds)
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            .6,
                            0,
                            0,
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
                    
            );
        } else if (getRunningTime() < 6.25) { //.5 seconds wait
        } else if (getRunningTime() < 9.5) { //Drive back onto charge station (for 3.25 seconds)
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            -.8,
                            0,
                            0,
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
                    
            );
        }
    }

    @Override
    public boolean isFinished() {
        return !shouldMove();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
/*
 Stages of command-based code:
 init (called 'initialize')
 periodic code - gets called in a loop (callled 'execute')
 cleanup (called 'end')

 extra functions to implement:
 public boolean isFinished() - signals to the CommandScheduler whether the command is done running
 */
}

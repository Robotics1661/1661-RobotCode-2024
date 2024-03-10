package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public record AutonomousInput(
    DrivetrainSubsystem drivetrainSubsystem,
    IntakeSubsystem intakeSubsystem,
    FourBarSubsystem fourBarSubsystem,
    ShooterSubsystem shooterSubsystem
) {
    public void stopAll() {
        drivetrainSubsystem.setControl(new SwerveRequest.Idle());
        intakeSubsystem.stop();
        fourBarSubsystem.endOfRoutineStop();
        shooterSubsystem.stop();
    }
}

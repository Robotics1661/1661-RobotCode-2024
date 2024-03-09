package frc.robot.subsystems;

public record AutonomousInput(
    DrivetrainSubsystem drivetrainSubsystem,
    IntakeSubsystem intakeSubsystem,
    FourBarSubsystem fourBarSubsystem,
    ShooterSubsystem shooterSubsystem
) {}

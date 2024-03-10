package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.commands.autos.pieces.AutoShooterCommand;
import frc.robot.commands.autos.pieces.DriveForwardCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand.EndBehaviour;
import frc.robot.commands.autos.pieces.SeedDrivetrainGyroCommand;
import frc.robot.commands.autos.pieces.SpoolUpShooterCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand;
import frc.robot.subsystems.AutonomousInput;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

// requires a preloaded note
public class SingleSpeakerShotAutoCommand {
    private static final double SPOOL_SPEED = -0.85;

    @INamedCommand("single_speaker_shot_auto")
    public static Command create(AutonomousInput autonomousInput) {
        /*
         * Sequence:
         *                          + Seed Drivetrain Gyro
         *                          |\
         *                          | \
         *       Go to SPEAKER pose +  \
         *                          |  + Start spooling up shooter
         *   Drive forward 1 second +  |
         *                          |  /
         *                          | /
         *                          |/
         *                          + Shoot into speaker
         *                          |
         *                          + Return to ORIGIN pose
         */
        return new SequentialCommandGroup(
            new SeedDrivetrainGyroCommand(autonomousInput.drivetrainSubsystem()), // init gyro
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new GoToSetPointCommand( // go to SPEAKER pose
                        autonomousInput.fourBarSubsystem(),
                        SetPoints.SPEAKER_AUTO,
                        EndBehaviour.Hold
                    ),
                    new DriveForwardCommand(autonomousInput.drivetrainSubsystem(), 1.0) // press against SPEAKER
                ),
                new SpoolUpShooterCommand(autonomousInput.shooterSubsystem(), SPOOL_SPEED)
            ),
            AutoShooterCommand.speaker( // shoot into SPEAKER
                autonomousInput.shooterSubsystem(),
                TimedIntakeCommand.makeScheduler(autonomousInput.intakeSubsystem())
            ).withInitialSpeed(SPOOL_SPEED),
            new GoToSetPointCommand( // go to ORIGIN pose
                autonomousInput.fourBarSubsystem(),
                SetPoints.ORIGIN,
                EndBehaviour.StaticBrake
            )
        );
    }
}

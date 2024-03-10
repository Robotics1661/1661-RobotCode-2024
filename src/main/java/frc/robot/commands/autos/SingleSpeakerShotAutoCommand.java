package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.commands.autos.pieces.AutoShooterCommand;
import frc.robot.commands.autos.pieces.DriveForwardCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand.EndBehaviour;
import frc.robot.commands.autos.pieces.SeedDrivetrainGyroCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand;
import frc.robot.subsystems.AutonomousInput;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

// requires a preloaded note
public class SingleSpeakerShotAutoCommand {
    @INamedCommand("single_speaker_shot_auto")
    public static Command create(AutonomousInput autonomousInput) {
        return new SequentialCommandGroup(
            new SeedDrivetrainGyroCommand(autonomousInput.drivetrainSubsystem()), // init gyro
            new GoToSetPointCommand( // go to SPEAKER pose
                autonomousInput.fourBarSubsystem(),
                SetPoints.SPEAKER_AUTO,
                EndBehaviour.Hold
            ),
            new DriveForwardCommand(autonomousInput.drivetrainSubsystem(), 1.0), // press against SPEAKER
            AutoShooterCommand.speaker( // shoot into SPEAKER
                autonomousInput.shooterSubsystem(),
                TimedIntakeCommand.makeScheduler(autonomousInput.intakeSubsystem())
            ),
            new GoToSetPointCommand( // go to ORIGIN pose
                autonomousInput.fourBarSubsystem(),
                SetPoints.ORIGIN,
                EndBehaviour.StaticBrake
            )
        );
    }
}

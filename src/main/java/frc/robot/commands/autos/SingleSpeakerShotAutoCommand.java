package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.pieces.AutoShooterCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand.EndBehaviour;
import frc.robot.subsystems.AutonomousInput;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

// requires a preloaded thingy
public class SingleSpeakerShotAutoCommand {
    public static Command create(AutonomousInput autonomousInput) {
        return new SequentialCommandGroup(
            new GoToSetPointCommand(
                autonomousInput.fourBarSubsystem(),
                SetPoints.SPEAKER,
                EndBehaviour.Hold
            ),
            AutoShooterCommand.speaker(
                autonomousInput.shooterSubsystem(),
                TimedIntakeCommand.makeScheduler(autonomousInput.intakeSubsystem())
            ),
            new GoToSetPointCommand(
                autonomousInput.fourBarSubsystem(),
                SetPoints.ORIGIN,
                EndBehaviour.StaticBrake
            )
        );
    }
}

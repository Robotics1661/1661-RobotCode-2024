package frc.robot.commands.named;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autos.pieces.AutoShooterCommand;
import frc.robot.commands.autos.pieces.FourBarStaticBrakeCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand.EndBehaviour;
import frc.robot.commands.autos.pieces.SpoolUpShooterCommand;
import frc.robot.commands.autos.pieces.TimedIntakeCommand;
import frc.robot.subsystems.AutonomousInput;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

public class MiscPieces {
    private static final double SPOOL_SPEED = -0.85;
    @INamedCommand("shoot_speaker_far_auto")
    public static Command shootSpeakerFarAuto(AutonomousInput autonomousInput) {
        return shootSpeakerFar(autonomousInput, true);
    }

    @INamedCommand("shoot_speaker_far_auto_norev")
    public static Command shootSpeakerFarAutoNoRev(AutonomousInput autonomousInput) {
        return shootSpeakerFar(autonomousInput, false);
    }

    private static Command shootSpeakerFar(AutonomousInput autonomousInput, boolean reverse) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new GoToSetPointCommand(
                    autonomousInput.fourBarSubsystem(),
                    SetPoints.FAR_SPEAKER_AUTO,
                    EndBehaviour.Hold,
                    true
                ),
                new SequentialCommandGroup(
                    reverse
                        ? IntakeCommand.onlyReverseSequence(autonomousInput.intakeSubsystem())
                        : new InstantCommand(),
                    new SpoolUpShooterCommand(
                        autonomousInput.shooterSubsystem(),
                        SPOOL_SPEED
                    )
                )
            ),
            new GoToSetPointCommand(
                autonomousInput.fourBarSubsystem(),
                SetPoints.FAR_SPEAKER_AUTO,
                EndBehaviour.Hold,
                true
            ),
            /*new InstantCommand(() -> {
                autoDbg("SHOOTING NOW!");
            })*/
            AutoShooterCommand.speaker(
                autonomousInput.shooterSubsystem(),
                TimedIntakeCommand.makeScheduler(autonomousInput.intakeSubsystem())
            ).withInitialSpeed(SPOOL_SPEED),
            new FourBarStaticBrakeCommand(autonomousInput.fourBarSubsystem()) // */
        );
    }
}

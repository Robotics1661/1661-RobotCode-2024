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
        return shootSpeakerFar(autonomousInput, true, SetPoints.FAR_SPEAKER_AUTO);
    }

    @INamedCommand("shoot_speaker_far_auto_norev")
    public static Command shootSpeakerFarAutoNoRev(AutonomousInput autonomousInput) {
        return shootSpeakerFar(autonomousInput, false, SetPoints.FAR_SPEAKER_AUTO);
    }

    @INamedCommand("shoot_speaker_far_auto_norev_amp_side")
    public static Command shootSpeakerFarAutoNoRevAmpSide(AutonomousInput autonomousInput) {
        return shootSpeakerFar(autonomousInput, false, SetPoints.FAR_SPEAKER_AUTO_AMP_SIDE);
    }

    @INamedCommand("shoot_speaker_far_auto_norev_source_side")
    public static Command shootSpeakerFarAutoNoRevSourceSide(AutonomousInput autonomousInput) {
        return shootSpeakerFar(autonomousInput, false, SetPoints.FAR_SPEAKER_AUTO_SOURCE_SIDE);
    }

    private static Command shootSpeakerFar(AutonomousInput autonomousInput, boolean reverse, SetPoints target) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new GoToSetPointCommand(
                    autonomousInput.fourBarSubsystem(),
                    target,
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
            /*new GoToSetPointCommand( // not needed (already in hold)
                autonomousInput.fourBarSubsystem(),
                target,
                EndBehaviour.Hold,
                true
            ),*/
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

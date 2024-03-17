package frc.robot.commands.named;

import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand;
import frc.robot.commands.autos.pieces.GoToSetPointCommand.EndBehaviour;
import frc.robot.subsystems.AutonomousInput;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

public class FourBarPieces {
    @INamedCommand("fourbar_intake")
    public static GoToSetPointCommand goToIntake(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.INTAKE,
            EndBehaviour.Hold,
            true
        );
    }

    @INamedCommand("fourbar_origin")
    public static GoToSetPointCommand goToOrigin(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.ORIGIN,
            EndBehaviour.StaticBrake,
            true
        );
    }

    @INamedCommand("fourbar_amp")
    public static GoToSetPointCommand goToAmp(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.AMP,
            EndBehaviour.Hold,
            false
        );
    }

    @INamedCommand("fourbar_far_shot")
    public static GoToSetPointCommand goToSpeakerFarShot(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.FAR_SPEAKER_AUTO,
            EndBehaviour.Hold,
            true
        );
    }

    @INamedCommand("fourbar_far_shot_amp_side")
    public static GoToSetPointCommand goToSpeakerFarShotAmpSide(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.FAR_SPEAKER_AUTO_AMP_SIDE,
            EndBehaviour.Hold,
            true
        );
    }

    @INamedCommand("fourbar_far_shot_source_side")
    public static GoToSetPointCommand goToSpeakerFarShotSourceSide(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.FAR_SPEAKER_AUTO_SOURCE_SIDE,
            EndBehaviour.Hold,
            4.0,
            true
        );
    }
}

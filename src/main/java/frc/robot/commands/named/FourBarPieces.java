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
            EndBehaviour.Hold
        );
    }

    @INamedCommand("fourbar_origin")
    public static GoToSetPointCommand goToOrigin(AutonomousInput autonomousInput) {
        return new GoToSetPointCommand(
            autonomousInput.fourBarSubsystem(),
            SetPoints.ORIGIN,
            EndBehaviour.StaticBrake
        );
    }
}

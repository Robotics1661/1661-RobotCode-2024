package frc.robot.commands.named;

import static frc.robot.util.AutonomousDebugger.autoDbg;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.AutonomousInput;

public class IntakePieces {
    @INamedCommand("start_intake")
    public static Command startIntake(AutonomousInput autonomousInput) {
        return new InstantCommand(() -> {
            autonomousInput.intakeSubsystem().run();
            autoDbg("Started intake");
        }, autonomousInput.intakeSubsystem());
    }

    @INamedCommand("momentary_reverse_intake")
    public static Command momentaryReverseIntake(AutonomousInput autonomousInput) {
        return IntakeCommand.onlyReverseSequence(autonomousInput.intakeSubsystem());
    }
}

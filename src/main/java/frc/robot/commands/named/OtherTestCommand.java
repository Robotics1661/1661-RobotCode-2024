package frc.robot.commands.named;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.subsystems.AutonomousInput;

public class OtherTestCommand {
    @INamedCommand("testme_other")
    public static Command testme(AutonomousInput ai) {
        return new InstantCommand();
    }
}

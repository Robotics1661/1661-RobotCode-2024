package frc.robot.commands.named;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.annotationprocessor.INamedCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OtherTestCommand {
    @INamedCommand("testme_other")
    public static Command testme(DrivetrainSubsystem d, FourBarSubsystem f, IntakeSubsystem i) {
        return new InstantCommand();
    }
}

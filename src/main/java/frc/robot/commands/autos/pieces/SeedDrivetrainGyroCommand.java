package frc.robot.commands.autos.pieces;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SeedDrivetrainGyroCommand extends InstantCommand {
    public SeedDrivetrainGyroCommand(DrivetrainSubsystem drivetrainSubsystem) {
        super(() -> drivetrainSubsystem.seedFieldRelative(), drivetrainSubsystem);
    }
}

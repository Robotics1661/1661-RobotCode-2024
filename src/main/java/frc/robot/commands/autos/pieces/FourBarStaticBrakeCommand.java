package frc.robot.commands.autos.pieces;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FourBarSubsystem;

public class FourBarStaticBrakeCommand extends InstantCommand {
    public FourBarStaticBrakeCommand(FourBarSubsystem fourBarSubsystem) {
        super(fourBarSubsystem::endOfRoutineStop, fourBarSubsystem);
    }
}

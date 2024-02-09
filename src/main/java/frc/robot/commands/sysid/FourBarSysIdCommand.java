package frc.robot.commands.sysid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.FourBarSubsystem;

public class FourBarSysIdCommand {
    @SuppressWarnings("deprecation")
    public static Command create(FourBarSubsystem fourBarSubsystem, BooleanSupplier safetySupplier) {
        return new SequentialCommandGroup(
            fourBarSubsystem.sysIdQuasistatic(Direction.kForward),
            fourBarSubsystem.sysIdQuasistatic(Direction.kReverse),
            fourBarSubsystem.sysIdDynamic(Direction.kForward),
            fourBarSubsystem.sysIdDynamic(Direction.kReverse)
        ).onlyWhile(safetySupplier);
    }
}
package frc.robot.commands.named;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@FunctionalInterface
public interface INamedCommandBuilder {
    Command build(DrivetrainSubsystem drivetrainSubsystem, FourBarSubsystem fourBarSubsystem, IntakeSubsystem intakeSubsystem);

    public static Optional<INamedCommandBuilder> match(Method method) {
        Class<?>[] provided = method.getParameterTypes();
        // NOTE: this must be updated if the signature of build() is updated
        Class<?>[] expected = new Class<?>[] {DrivetrainSubsystem.class, FourBarSubsystem.class, IntakeSubsystem.class};

        if (!Command.class.isAssignableFrom(method.getReturnType())) {
            return Optional.empty();
        }

        if (expected.length != provided.length) {
            return Optional.empty();
        }

        for (int i = 0; i < expected.length; i++) {
            if (!expected[i].isAssignableFrom(provided[i])) {
                return Optional.empty();
            }
        }

        if (!method.trySetAccessible()) {
            return Optional.empty();
        }

        return Optional.of((driveTrain, fourBar, intake) -> {
            try {
                return (Command) method.invoke(null, driveTrain, fourBar, intake);
            } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                if (Utils.isSimulation()) {
                    throw new RuntimeException("(Simulation only) named command creation failed", e);
                }
                e.printStackTrace();
                return new InstantCommand();
            }
        });
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

import java.util.function.BooleanSupplier;

public class ClawCommand extends CommandBase {

    private final ClawSubsystem m_clawSubsystem;

    private final BooleanSupplier m_coneSupplier;
    private final BooleanSupplier m_cubeSupplier;
    private final BooleanSupplier m_releaseSupplier;
    private final BooleanSupplier m_offSupplier;

    public ClawCommand(ClawSubsystem clawSubsystem,
                               BooleanSupplier coneSupplier,
                               BooleanSupplier cubeSupplier,
                               BooleanSupplier releaseSupplier,
                               BooleanSupplier offSupplier) {
        this.m_clawSubsystem = clawSubsystem;

        this.m_coneSupplier = coneSupplier;
        this.m_cubeSupplier = cubeSupplier;
        this.m_releaseSupplier = releaseSupplier;
        this.m_offSupplier = offSupplier;

        addRequirements(clawSubsystem);
    }

    private double solenoidNeutralTime = 0;
    private boolean neutralizeSolenoid = false;

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        if (neutralizeSolenoid && now >= solenoidNeutralTime) {
            neutralizeSolenoid = false;
            m_clawSubsystem.setNeutral();
        }

        boolean changed = false;
        if (m_coneSupplier.getAsBoolean()) {
            m_clawSubsystem.grabCone();
            changed = true;
        } else if (m_cubeSupplier.getAsBoolean()) {
            m_clawSubsystem.grabCube();
            changed = true;
        } else if (m_releaseSupplier.getAsBoolean()) {
            m_clawSubsystem.release();
            changed = true;
        } else if (m_offSupplier.getAsBoolean()) {
            m_clawSubsystem.turnOff();
        }

        if (changed) {
            neutralizeSolenoid = true;
            solenoidNeutralTime = now + 0.5;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_clawSubsystem.release();
        m_clawSubsystem.turnOff();
    }
}

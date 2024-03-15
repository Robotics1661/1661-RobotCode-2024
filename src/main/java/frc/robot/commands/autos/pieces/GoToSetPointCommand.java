package frc.robot.commands.autos.pieces;

import static frc.robot.util.AutonomousDebugger.autoDbg;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.FourBarSubsystem.SetPoints;

public class GoToSetPointCommand extends Command {
    private final FourBarSubsystem m_fourBarSubsystem;
    private final SetPoints m_target;
    private final EndBehaviour m_endBehaviour;
    private final double m_precision;
    private final boolean m_faster;
    
    private double startTime;

    public GoToSetPointCommand(
        FourBarSubsystem fourBarSubsystem,
        SetPoints target,
        EndBehaviour endBehaviour
    ) {
        this(fourBarSubsystem, target, endBehaviour, 0.75, false);
    }

    public GoToSetPointCommand(
        FourBarSubsystem fourBarSubsystem,
        SetPoints target,
        EndBehaviour endBehaviour,
        double precision
    ) {
        this(fourBarSubsystem, target, endBehaviour, precision, false);
    }

    public GoToSetPointCommand(
        FourBarSubsystem fourBarSubsystem,
        SetPoints target,
        EndBehaviour endBehaviour,
        boolean faster
    ) {
        this(fourBarSubsystem, target, endBehaviour, 0.75, faster);
    }

    public GoToSetPointCommand(
        FourBarSubsystem fourBarSubsystem,
        SetPoints target,
        EndBehaviour endBehaviour,
        double precision,
        boolean faster
    ) {
        this.m_fourBarSubsystem = fourBarSubsystem;
        this.m_target = target;
        this.m_endBehaviour = endBehaviour;
        this.m_precision = precision;
        this.m_faster = faster;
    }

    @Override
    public void initialize() {
        super.initialize();

        m_fourBarSubsystem.setTargetPoint(m_target, m_faster);
        startTime = Timer.getFPGATimestamp();

        autoDbg("[g2sp] Beginning traversal to "+m_target.name());
    }

    @Override
    public boolean isFinished() {
        if (Utils.isSimulation()) {
            return Timer.getFPGATimestamp() >= startTime + 2.0;
        }
        if (Timer.getFPGATimestamp() < startTime + 0.5) return false;
        if (m_fourBarSubsystem.isClosedLoopErrorWithin(m_precision)) {
            autoDbg("[g2sp] closed loop ok");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (interrupted) {
            m_fourBarSubsystem.endOfRoutineStop();
            autoDbg("[g2sp] StaticBrake (interrupted)");
        } else {
            switch (m_endBehaviour) {
                case Hold -> {
                    autoDbg("[g2sp] Hold "+m_target.name());
                }
                case ActiveBrake -> {
                    m_fourBarSubsystem.activeStop();
                    autoDbg("[g2sp] ActiveBrake");
                }
                case StaticBrake -> {
                    m_fourBarSubsystem.endOfRoutineStop();
                    autoDbg("[g2sp] StaticBrake");
                }
            }
        }
    }

    public static enum EndBehaviour {
        Hold,
        ActiveBrake,
        StaticBrake
    }
}

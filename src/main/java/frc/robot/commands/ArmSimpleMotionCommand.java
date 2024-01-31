package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmSimpleMotionCommand extends CommandBase {
    private final ArmSubsystem m_armSubsystem;

    private final DoubleSupplier m_rotationElbowSupplier;
    private final DoubleSupplier m_rotationShoulderSupplier;
    private final BooleanSupplier m_bypassLimitSupplier;

    public ArmSimpleMotionCommand(ArmSubsystem armSubsystem,
                               DoubleSupplier rotationElbowSupplier,
                               DoubleSupplier rotationShoulderSupplier,
                               BooleanSupplier bypassLimitSupplier) {
        this.m_armSubsystem = armSubsystem;
        this.m_rotationElbowSupplier = rotationElbowSupplier;
        this.m_rotationShoulderSupplier = rotationShoulderSupplier;
        this.m_bypassLimitSupplier = bypassLimitSupplier;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double elbow = m_rotationElbowSupplier.getAsDouble();
        SmartDashboard.putNumber("elbowControlDebug", elbow);
        m_armSubsystem.moveElbow(elbow);
        m_armSubsystem.moveShoulder(m_rotationShoulderSupplier.getAsDouble(), !m_bypassLimitSupplier.getAsBoolean());
        m_armSubsystem.dashboardInfo();
        //if (m_zeroSensorSupplier.getAsBoolean())
        //    m_armSubsystem.zeroMotorSensors();
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.moveElbow(0);
        m_armSubsystem.moveShoulder(0, false);
    }
}

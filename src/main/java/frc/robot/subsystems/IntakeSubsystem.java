package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_ID;
import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("removal") // have to use phoenix 5 for talon control
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    //private final DutyCycleOut m_request;

    //private static final double MAX_VOLTAGE = 12;

    public IntakeSubsystem() {
        m_motor = new TalonFX(INTAKE_ID);
        //m_request = new DutyCycleOut(0);

        /*TalonFXConfigurator config = m_motor.getConfigurator();
        config.apply(new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(-MAX_VOLTAGE)
        );*/

        m_motor.setNeutralMode(NeutralMode.Brake);

        stop();
    }

    private void setSpeed(double percent) {
        percent = clamp(percent, -1, 1);
        m_motor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void run() {
        setSpeed(0.85);
    }

    public void stop() {
        setSpeed(0);
        m_motor.set(TalonFXControlMode.Disabled, 0);
    }
}
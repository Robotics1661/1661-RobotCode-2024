package frc.robot.subsystems;

import static frc.robot.Constants.CLIMBER_ID;
import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    private static final double MAX_VOLTAGE = 12.0;

    private final DutyCycleOut m_request = new DutyCycleOut(0.0);
    private final StaticBrake m_brake = new StaticBrake();

    public ClimberSubsystem() {
        m_motor = new TalonFX(CLIMBER_ID);

        TalonFXConfigurator config = m_motor.getConfigurator();
        config.apply(new VoltageConfigs()
            .withPeakForwardVoltage(MAX_VOLTAGE)
            .withPeakReverseVoltage(-MAX_VOLTAGE));
    }

    private void setSpeed(double target) {
        target = clamp(target, -1.0, 1.0);
        if (Math.abs(target) < 0.001) {
            m_motor.setControl(m_brake);
        } else {
            m_motor.setControl(m_request.withOutput(target));
        }
    }

    public void extend() {
        setSpeed(0.8);
    }

    public void retract() {
        setSpeed(-0.5);
    }

    public void stop() {
        setSpeed(0.0);
    }
}

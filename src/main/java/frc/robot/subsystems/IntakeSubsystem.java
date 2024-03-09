package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_ID;
import static frc.robot.util.MathUtil.clamp;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor;

    private double currentSpeed = 0;
    private double targetSpeed = 0;
    private boolean lerpMode = true;

    public IntakeSubsystem() {
        m_motor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.setSmartCurrentLimit(80);

        stop();
    }

    private void setSpeed(double targetPercent) {
        targetSpeed = clamp(targetPercent, -1, 1);
        lerpMode = true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (lerpMode) {
            currentSpeed = MathUtil.lerp(currentSpeed, targetSpeed, 0.1);
            if (Math.abs(currentSpeed) < 0.01) {
                currentSpeed = 0;
            }
            if (Math.abs(currentSpeed) > 0.99) {
                currentSpeed = 1 * Math.signum(currentSpeed);
            }

            m_motor.set(currentSpeed);
        }
    }

    public void run() {
        setSpeed(0.95); // was 0.85
    }

    public void reverse(double percent) {
        double speed = clamp(percent, 0, 1) * -0.4;
        currentSpeed = speed;
        targetSpeed = speed;
        m_motor.set(speed);
        lerpMode = false;
    }

    public void stop() {
        setSpeed(0);
    }
}
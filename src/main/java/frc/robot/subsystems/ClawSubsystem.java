package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid solenoid_6_inch;
    private final DoubleSolenoid solenoid_7_inch;
    private Mode mode;

    public ClawSubsystem() {
        solenoid_6_inch = new DoubleSolenoid(Constants.PCM, PneumaticsModuleType.CTREPCM, Constants.SOLENOID_6_INCH_FORWARD, Constants.SOLENOID_6_INCH_REVERSE);
        solenoid_7_inch = new DoubleSolenoid(Constants.PCM, PneumaticsModuleType.CTREPCM, Constants.SOLENOID_7_INCH_FORWARD, Constants.SOLENOID_7_INCH_REVERSE);
        mode = Mode.OPEN;
    }

    public void setNeutral() {
        solenoid_6_inch.set(Value.kOff);
        solenoid_7_inch.set(Value.kOff);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        //apply mode
        solenoid_6_inch.set(this.mode.mode_6_inch);
        solenoid_7_inch.set(this.mode.mode_7_inch);

        SmartDashboard.putString("clawMode", this.mode.name());
    }

    public Mode getMode() {
        return this.mode;
    }

    public boolean isOpen() {
        return this.mode == Mode.OPEN;
    }

    public void grabCone() {
        setMode(Mode.CONE);
    }

    public void grabCube() {
        setMode(Mode.CUBE);
    }

    public void release() {
        setMode(Mode.OPEN);
    }

    public void turnOff() {
        setMode(Mode.OFF);
    }

    public enum Mode {
        OPEN(Value.kForward, Value.kForward), //Forward is open, Reverse is closed
        CONE(Value.kReverse, Value.kReverse),
        CUBE(Value.kForward, Value.kReverse),
        OFF(Value.kOff, Value.kOff)
        ;
        public final Value mode_6_inch;
        public final Value mode_7_inch;

        private Mode(Value mode_6_inch, Value mode_7_inch) {
            this.mode_6_inch = mode_6_inch;
            this.mode_7_inch = mode_7_inch;
        }
    }
}

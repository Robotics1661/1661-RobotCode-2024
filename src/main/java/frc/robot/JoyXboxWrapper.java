package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoyXboxWrapper {
    private final XboxController m_xbox;
    private final Joystick m_joystick; //flightstick
    private boolean m_soft_disabled;
    private final boolean m_auto_swap_mode = true; // If true, soft enable if twist is on +

    public JoyXboxWrapper(int xboxPort, int joystickPort) {
        this(xboxPort, joystickPort, false);
    }

    public JoyXboxWrapper(int xboxPort, int joystickPort, boolean controllerMode) {
        m_xbox = new XboxController(xboxPort);
        m_joystick = new Joystick(joystickPort);
        m_soft_disabled = controllerMode;
    }

    public double getLateralX() {
        return 0;//isSoftDisabled() ? 0 : m_joystick.getX();
    }

    public double getLateralY() {
        return isSoftDisabled() ? 0 : m_joystick.getY();
    }

    public double getRotation() {
        return isSoftDisabled() ?
        0 :
        (m_joystick.getTrigger() ? m_joystick.getZ() : 0);
    }

    private boolean getFlightButton(int id) {
        return isSoftDisabled() ? false : m_joystick.getRawButton(id);
    }

    public boolean getZeroButton() {
        return getFlightButton(7);
    }

    public double getFourBarSpeed() {
        if (isSoftDisabled()) return 0;
        return m_xbox.getRightY();
    }

    public boolean getTestFourBar() {
        return getFlightButton(11);
    }

    public boolean getIntake() {
        return getFlightButton(12);
    }

    public void softDisable() {
        SmartDashboard.putString("controller_mode", "Soft Disabled");
        this.m_soft_disabled = true;
    }

    public void softEnable() {
        SmartDashboard.putString("controller_mode", "Soft Enabled");
        this.m_soft_disabled = false;
    }

    public boolean isSoftDisabled() {
        if (this.m_auto_swap_mode) {
            SmartDashboard.putNumber("twist", this.m_joystick.getThrottle());
            if (this.m_joystick.getThrottle() < -0.75) {
                this.softEnable();
            } else if (this.m_joystick.getThrottle() > 0.75) {
                this.softDisable();
            }
        }
        return this.m_soft_disabled;
    }
}

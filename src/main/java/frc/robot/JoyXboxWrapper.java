package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoyXboxWrapper {
    private final XboxController m_xbox;
    private final Joystick m_joystick; //flightstick
    private boolean m_soft_disabled;
    private final boolean m_auto_swap_mode = true; // If true, soft enable if twist is on +
    private final boolean swapElbowShoulderSticks = false;

    public JoyXboxWrapper(int xboxPort, int joystickPort) {
        this(xboxPort, joystickPort, false);
    }

    public JoyXboxWrapper(int xboxPort, int joystickPort, boolean controllerMode) {
        m_xbox = new XboxController(xboxPort);
        m_joystick = new Joystick(joystickPort);
        m_soft_disabled = controllerMode;
    }

    public boolean getBalance() {
        return getFlightButton(4);
    }

    public boolean getBypassLimitButton() {
        return getFlightButton(11);
    }

    public double getLateralX() {
        return isSoftDisabled() ? 0 : m_joystick.getX();
    }

    public double getLateralY() {
        return isSoftDisabled() ? 0 : m_joystick.getY();
    }

    public double getRotation() {
        return isSoftDisabled() ?
        0 :
        (m_joystick.getTrigger() ? m_joystick.getZ() : 0);
    }

    protected boolean getFlightButton(int id) {
        return isSoftDisabled() ? false : m_joystick.getRawButton(id);
    }

    public boolean getZeroButton() {
        return getFlightButton(7);
    }

    public double getElbowRotation() {
        if (isSoftDisabled()) return 0;
        return swapElbowShoulderSticks ? m_xbox.getLeftY() : m_xbox.getRightY();
    }

    public double getShoulderRotation() {
        if (isSoftDisabled()) return 0;
        return swapElbowShoulderSticks ? m_xbox.getRightY() : m_xbox.getLeftY();
    }

    public boolean getClawCone() {
        return getFlightButton(Constants.CONE_BUTTON);
    }
    
    public boolean getClawCube() {
        return getFlightButton(Constants.CUBE_BUTTON);
    }

    public boolean getClawRelease() {
        return getFlightButton(Constants.RELEASE_BUTTON);
    }

    public boolean getClawOff() {
        return isSoftDisabled();
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

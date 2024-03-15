package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoyXboxWrapper {
    private final Optional<XboxController> m_xbox;
    private final Optional<Joystick> m_joystick; //flightstick
    @SuppressWarnings("unused")
    private boolean m_soft_disabled;
    @SuppressWarnings("unused")
    private final boolean m_auto_swap_mode = true; // If true, soft enable if twist is on +

    public JoyXboxWrapper(int xboxPort, int joystickPort) {
        this(xboxPort, joystickPort, false);
    }

    public JoyXboxWrapper(int xboxPort, int joystickPort, boolean controllerMode) {
        if (Utils.isSimulation()) {
            m_xbox = Optional.empty();
            m_joystick = Optional.empty();
        } else {
            m_xbox = Optional.of(new XboxController(xboxPort));
            m_joystick = Optional.of(new Joystick(joystickPort));
        }
        m_soft_disabled = controllerMode;
    }

    public double getLateralX() {
        if (Constants.DISABLE_SWERVE) return 0;
        return isSoftDisabled() ? 0 : m_joystick.map(Joystick::getX).orElse(0.0);
    }

    public double getLateralY() {
        if (Constants.DISABLE_SWERVE) return 0;
        return isSoftDisabled() ? 0 : m_joystick.map(Joystick::getY).orElse(0.0);
    }

    public double getRotation() {
        if (Constants.DISABLE_SWERVE) return 0;
        return isSoftDisabled() ?
        0 :
        (m_joystick.map(Joystick::getTrigger).orElse(false)
            ? m_joystick.map(Joystick::getZ).orElse(0.0)
            : 0
        );
    }

    private boolean getFlightButton(int id) {
        return isSoftDisabled()
            ? false
            : m_joystick.map(j -> j.getRawButton(id)).orElse(false);
    }

    public boolean getZeroButton() {
        return getFlightButton(7);
    }

    public boolean getFourBarInitialize() {
        return getFlightButton(8);
    }

    public boolean getBrakeButton() {
        return getFlightButton(9);
    }

    public boolean getZeroButton180() {
        return getFlightButton(10);
    }

    public boolean getSysIdSafety() {
        return getFlightButton(11);
    }

    public boolean getIntake() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getRightBumper).orElse(false);
        //return getFlightButton(5);
    }

    public double getIntakeReverse() {
        if (isSoftDisabled()) return 0.0;
        return m_xbox.map(XboxController::getRightTriggerAxis).orElse(0.0);
    }

    public double getFourBarSpeed() {
        if (isSoftDisabled()) return 0;
        return m_xbox.map(XboxController::getRightY).orElse(0.0);
    }

    public boolean getFourBarOrigin() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getYButton).orElse(false);        
    }

    public boolean getFourBarIntake() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getBButton).orElse(false);
    }

    public boolean getFourBarAmp() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getXButton).orElse(false);
    }

    public boolean getFourBarSpeaker() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getAButton).orElse(false);
    }

    public boolean getFourBarFarSpeaker() {
        return getFlightButton(12);
    }

    public boolean getClimberExtend() {
        if (isSoftDisabled()) return false;
        int pov = m_joystick.map(Joystick::getPOV).orElse(-1);
        if (pov == -1) return false;
        return pov == 0 || pov == 45 || pov == 315;
    }

    public boolean getClimberRetract() {
        if (isSoftDisabled()) return false;
        int pov = m_joystick.map(Joystick::getPOV).orElse(-1);
        if (pov == -1) return false;
        return pov == 180 || pov == 180+45 || pov == 180-45;
    }

    public boolean getTestShooterSpeedIncrease() {
        if (isSoftDisabled()) return false;
        int pov = m_xbox.map(XboxController::getPOV).orElse(-1);
        if (pov == -1) return false;
        return pov == 0;
    }

    public boolean getTestShooterSpeedDecrease() {
        if (isSoftDisabled()) return false;
        int pov = m_xbox.map(XboxController::getPOV).orElse(-1);
        if (pov == -1) return false;
        return pov == 180;
    }

    public boolean getAmpShot() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getLeftBumper).orElse(false);
    }

    public boolean getSpeakerShot() {
        if (isSoftDisabled()) return false;
        return m_xbox.map(XboxController::getLeftTriggerAxis).orElse(0.0) > 0.5;
    }

    /*public double getShooterSpeed() {
        if (isSoftDisabled()) return 0;
        if (m_xbox.map(XboxController::getLeftBumper).orElse(false)) return -Math.sqrt(0.45); // -1 for general shooting, -sqrt(0.45) for amp
        return m_xbox.map(XboxController::getLeftY).orElse(0.0);
    }*/

    public void softDisable() {
        SmartDashboard.putString("controller_mode", "Soft Disabled");
        this.m_soft_disabled = true;
    }

    public void softEnable() {
        SmartDashboard.putString("controller_mode", "Soft Enabled");
        this.m_soft_disabled = false;
    }

    public boolean isSoftDisabled() {
        return false; // FIXME: temporary to allow joystick-less use
        /*if (this.m_auto_swap_mode) {
            SmartDashboard.putNumber("twist", this.m_joystick.getThrottle());
            if (this.m_joystick.getThrottle() < -0.75) {
                this.softEnable();
            } else if (this.m_joystick.getThrottle() > 0.75) {
                this.softDisable();
            }
        }
        return this.m_soft_disabled;*/
    }
}

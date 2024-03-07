package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;

public class TimeUtil {
    public static void busySleep(long milliseconds) {
        long end = RobotController.getFPGATime() + (milliseconds * 1000);
        while (RobotController.getFPGATime() < end) {}
    }
}

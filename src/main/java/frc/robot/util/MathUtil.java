package frc.robot.util;

public abstract class MathUtil {
    public static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }

    public static double degreesToRadians(double degrees) {
        return (degrees / 180) * Math.PI;
    }
}

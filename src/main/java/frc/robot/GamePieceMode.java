package frc.robot;

public enum GamePieceMode {
    CONE(58.740087, 0.0, 0.0), //FIXME real values (or don't, not an auton we care about)
    CUBE(58.740087, 75.945737, 25)
    ;
    public final double elbowPrelimDegrees;
    public final double shoulderTargetDegrees;
    public final double elbowTargetDegrees;
    GamePieceMode(double elbowPrelimDegrees, double shoulderTargetDegrees, double elbowTargetDegrees) {
        this.elbowPrelimDegrees = elbowPrelimDegrees;
        this.shoulderTargetDegrees = shoulderTargetDegrees;
        this.elbowTargetDegrees = elbowTargetDegrees;
    }
}

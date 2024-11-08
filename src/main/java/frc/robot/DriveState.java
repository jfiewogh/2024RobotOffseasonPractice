package frc.robot;

public class DriveState {
    private double longitudinalPosition;
    private double lateralPosition;
    private double angleRadians;
    private int points;

    public DriveState(double longitudinalPosition, double lateralPosition, double angleRadians, int points) {
        this.longitudinalPosition = longitudinalPosition;
        this.lateralPosition = lateralPosition;
        this.angleRadians = angleRadians;
        this.points = points;
    }

    public double getLongitudinalPosition() {
        return longitudinalPosition;
    }
    public double getLateralPosition() {
        return lateralPosition;
    }
    public double getAngleRadians() {
        return angleRadians;
    }
    public int getPoints() {
        return points;
    }
}

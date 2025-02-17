package org.firstinspires.ftc.teamcode.CustomDriveKinematics;

public class MecanumWheel {
    private final double locationX;
    private final double locationY;
    private final double wheelRadius;
    MecanumWheel(double XOffset, double YOffset, double wheelRadius){
        this.locationX = XOffset;
        this.locationY = YOffset;
        this.wheelRadius = wheelRadius;
    }
    public double getLocationX() {
        return locationX;
    }
    public double getLocationY() {
        return locationY;
    }
    public double getWheelRadius() {
        return wheelRadius;
    }
}

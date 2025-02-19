package org.firstinspires.ftc.teamcode.byteLibrary.classes;

public class MecanumWheel{
    private final double locationX;
    private final double locationY;
    private final double wheelRadius;
    private final MotorController controller;
    public MecanumWheel(double XOffset, double YOffset, double wheelRadius, MotorController motor){
        this.locationX = XOffset;
        this.locationY = YOffset;
        this.wheelRadius = wheelRadius;
        this.controller = motor;
    }
    public void setMotorPower(double wheelSpeed) {
        controller.setMotorPower(wheelSpeed);
    }
    public void setMotorVelocity(double velocity) {controller.setMotorVelocity(velocity);}
    public double getMotorVelocity() {return controller.getMotorVelocity();}
    public int getCurrentPosition() {
        return controller.getCurrentPosition();
    }
    public double getMotorPower(){
        return controller.getMotorPower();
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

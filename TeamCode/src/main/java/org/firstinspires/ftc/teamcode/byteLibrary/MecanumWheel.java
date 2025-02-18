package org.firstinspires.ftc.teamcode.byteLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumWheel{
    private final double locationX;
    private final double locationY;
    private final double wheelRadius;
    private final MotorController controller;
    MecanumWheel(double XOffset, double YOffset, double wheelRadius, MotorController motor){
        this.locationX = XOffset;
        this.locationY = YOffset;
        this.wheelRadius = wheelRadius;
        this.controller = motor;
    }
    public void setMotorPower(double wheelSpeed) {
        controller.setMotorPower(wheelSpeed);
    }
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

package org.firstinspires.ftc.teamcode.CustomDriveKinematics;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumWheel {
    private final double locationX;
    private final double locationY;
    private final double wheelRadius;
    private final DcMotor motor;
    MecanumWheel(double XOffset, double YOffset, double wheelRadius, DcMotor motor){
        this.locationX = XOffset;
        this.locationY = YOffset;
        this.wheelRadius = wheelRadius;
        this.motor = motor;
    }
    public void setMotorPosition(int position){
        motor.setTargetPosition(position);
    }
    public void setMotorPower(double power){
        motor.setPower(power);
    }
    public void setMotorRunMode(DcMotor.RunMode mode){
        motor.setMode(mode);
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
    public double getMotorPower(){
        return motor.getPower();
    }
    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }
}

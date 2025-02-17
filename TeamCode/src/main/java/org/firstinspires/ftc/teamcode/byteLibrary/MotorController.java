package org.firstinspires.ftc.teamcode.byteLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorController {
    private DcMotor motor;
    private final String deviceName;
    MotorController(DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior behavior, DcMotorSimple.Direction dir, String deviceName){
        this.motor.setMode(runMode); this.motor.setDirection(dir); this.motor.setZeroPowerBehavior(behavior);
        this.deviceName = deviceName;
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
    public double getMotorPower(){
        return motor.getPower();
    }
    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }
    public DcMotor getMotor() {
        return motor;
    }
    public String getDeviceName() {
        return deviceName;
    }
}

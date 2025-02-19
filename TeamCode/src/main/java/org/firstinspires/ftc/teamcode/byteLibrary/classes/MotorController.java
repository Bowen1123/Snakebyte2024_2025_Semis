package org.firstinspires.ftc.teamcode.byteLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class MotorController {
    private final DcMotor motor;
    private final String deviceName;
    private final HardwareMap map;
    MotorController(DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior behavior, DcMotorSimple.Direction dir, String deviceName, HardwareMap map){
        this.deviceName = deviceName;
        this.map = map;
        this.motor = this.map.get(DcMotor.class, this.deviceName);
        this.motor.setMode(runMode); this.motor.setDirection(dir); this.motor.setZeroPowerBehavior(behavior);
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
    public HardwareMap getMap(){return map;}
}

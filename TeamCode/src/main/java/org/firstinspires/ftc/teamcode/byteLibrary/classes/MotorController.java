package org.firstinspires.ftc.teamcode.byteLibrary.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class MotorController {
    private final DcMotorEx motor;
    private final DcMotor.RunMode runMode;
    private final String deviceName;
    private final HardwareMap map;

    public MotorController(DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior behavior, DcMotorSimple.Direction dir, String deviceName, HardwareMap map){
        this.deviceName = deviceName;
        this.runMode = runMode;
        this.map = map;
        this.motor = this.map.get(DcMotorEx.class, this.deviceName);
        this.motor.setMode(runMode); this.motor.setDirection(dir); this.motor.setZeroPowerBehavior(behavior);
    }
    public MotorController(DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior behavior, DcMotorSimple.Direction dir, String deviceName, HardwareMap map, PIDFCoefficients velpid){
        this.deviceName = deviceName;
        this.runMode = runMode;
        this.map = map;
        this.motor = this.map.get(DcMotorEx.class, this.deviceName);
        this.motor.setMode(runMode); this.motor.setDirection(dir); this.motor.setZeroPowerBehavior(behavior);
        this.motor.setVelocityPIDFCoefficients(velpid.p, velpid.i, velpid.d, velpid.f);
    }
    public void setPIDF(PIDFCoefficients pidf){
        this.motor.setPIDFCoefficients(this.runMode, pidf);
    }
    public void setPIDF(double p){
        this.motor.setPositionPIDFCoefficients(p);
    }
    public void setMotorPosition(int position){
        motor.setTargetPosition(position);
    }
    public void setMotorVelocity(double velocity) { motor.setVelocity(velocity); }
    public void setMotorPower(double power){
        motor.setPower(power);
    }
    public void setMotorRunMode(DcMotor.RunMode mode){
        motor.setMode(mode);
    }
    public double getMotorPower(){
        return motor.getPower();
    }
    public double getMotorVelocity() {return motor.getVelocity();}
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

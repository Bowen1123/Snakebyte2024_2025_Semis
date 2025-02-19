package org.firstinspires.ftc.teamcode.byteLibrary.classes;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ElevatorController {
    private final MotorController controller;
    private final double gearRatio;
    private final PIDFCoefficients pidf;
    public ElevatorController(MotorController motor, double sensorToMech, PIDFCoefficients pidf){
        this.controller = motor;
        this.gearRatio = sensorToMech;
        this.pidf = pidf;
    }
    public void setElevatorVelocity(double vel){
        this.controller.setMotorVelocity(vel);
    }
    public void setElevatorPosition(double mechPos){
        this.controller.setMotorPosition(convertMechSpaceToWheelTicks(mechPos));
    }
    public void setPIDFToMotor(){
        this.controller.setPIDF(this.pidf);
    }
    public void setPToMotor(){
        this.controller.setPIDF(this.pidf.p);
    }
    public double convertWheelTicksToMechSpace(int ticks){
        // sensor : mech, sensor : 1, we want in mech space
        return ticks * (1/gearRatio);
    }
    public int convertMechSpaceToWheelTicks(double mechSpace){
        return (int) Math.round(mechSpace * gearRatio);
    }
    public void setP(double p){
        this.pidf.p = p;
    }
    public void setI(double i){
        this.pidf.i = i;
    }
    public void setD(double d){
        this.pidf.d = d;
    }
    public void setFeedforward(double f){
        this.pidf.f = f;
    }
    public MotorController getController() {
        return controller;
    }
    public double getGearRatio() {
        return gearRatio;
    }
    public PIDFCoefficients getPidf() {
        return pidf;
    }
}

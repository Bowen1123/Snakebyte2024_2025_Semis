package org.firstinspires.ftc.teamcode.byteLibrary;

public class ElevatorController {
    private final MotorController controller;
    private final double gearRatio;
    ElevatorController(MotorController motor, double gearRatio){
        this.controller = motor;
        this.gearRatio = gearRatio;
    }
    public MotorController getController() {
        return controller;
    }
    public double getGearRatio() {
        return gearRatio;
    }
}

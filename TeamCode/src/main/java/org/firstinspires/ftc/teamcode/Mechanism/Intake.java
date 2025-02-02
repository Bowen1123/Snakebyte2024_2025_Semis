package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake{
    private DcMotor horizonatal;
    private CRServo spinner;
    private TouchSensor sensor;
    private Servo wrist;
    private boolean init, eaten, slideExtended;

    public Intake(HardwareMap hardwareMap){
        horizonatal = hardwareMap.get(DcMotorEx.class, "horizontal");
        horizonatal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizonatal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");

        spinner = hardwareMap.get(CRServo.class, "spinner");
        // eater = hardwareMap.get(TouchSensor.class, "eater");

        init = true;
        slideExtended = false;
    }

    /*public Action retract(){
        return new Retract();
    }*/
    public Action extend() {return new Extend(); }

    public Action wristDown() { return new WristDown(); }
    public Action wristUp() {return new WristUp(); }


    public class WristDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = 1;
            wrist.setPosition(targetPos /2);
            wrist.setPosition(targetPos - .1);
            wrist.setPosition(targetPos);
            return false;
        }
    }
    public class WristUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .3;
            wrist.setPosition(targetPos);
            return false;
        }
    }
    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = horizonatal.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos < 3000.0) {
                horizonatal.setTargetPosition(3000);
                slideExtended = true;
                return true;
            } else {
                return false;
            }
        }
    }

    public class Extend implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(horizonatal.getCurrentPosition());
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos < 1500) {
                while (pos < 1500){
                    horizonatal.setTargetPosition(1500);
                    horizonatal.setPower(.4);
                    pos = Math.abs(horizonatal.getCurrentPosition());
                }
                return false;
            } else {
                return false;
            }
        }
    }


}

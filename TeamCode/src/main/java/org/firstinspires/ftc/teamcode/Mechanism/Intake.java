package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake{
    public  DcMotor horizontal;

    private CRServo spinner;
    //private TouchSensor sensor;
    private Servo wrist;
    private TouchSensor touchSensor;
    private boolean init, eaten, slideExtended;
    private double time = 0;
    private int targetPosition;
    private ElapsedTime timer;
    private String status = "";
    private String wristStatus = "";
    public Intake (){
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner = hardwareMap.get(CRServo.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist = hardwareMap.get(Servo.class, "wrist");

        touchSensor = hardwareMap.get(TouchSensor.class, "sensor");
    }

    public Intake(HardwareMap hardwareMap){
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner = hardwareMap.get(CRServo.class, "spinner");
        wrist = hardwareMap.get(Servo.class, "wrist");

        timer = new ElapsedTime();

    }

    /*public Action retract(){
        return new Retract();
    }*/
    public Action extend() {return new Extend(); }
    public Action retract() {return new Retract(); }
    public Action wristDown() { return new WristDown(); }
    public Action wristUp() {return new WristUp(); }
    public Action wristSemi() {return new WristSemi(); }
    public Action wristVertical() {return new WristVertical(); }
    public Action wristTravel() {return new WristTravel(); }
    public Action retractMid() {return new RetractMid(); }
    public Action spinner(double time){
        timer.reset();
        this.time = time;

        return new SpinnerTime();
    }

    public Action activateSpinner(){
        return new ActivateSpinner();
    }
    public class ActivateSpinner implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinner.setPower(-1);
            return false;
        }
    }

    public class DeactivateSpinner implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinner.setPower(0);
            return false;
        }
    }


    public Action deactivateSpinner(){
        return new DeactivateSpinner();
    }

    public Action goToPos(int targetPosition){
        this.targetPosition = targetPosition;

        return new GoToPos();
    }

    public class GoToPos implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(horizontal.getCurrentPosition());
            if (pos < targetPosition - 10) {
                horizontal.setTargetPosition(targetPosition);
                horizontal.setPower(.8);
                pos = Math.abs(horizontal.getCurrentPosition());
                return true;
            } else if (pos > targetPosition + 10){
                horizontal.setTargetPosition(targetPosition);
                horizontal.setPower(-.8);
                pos = Math.abs(horizontal.getCurrentPosition());
                return true;
            } else {
                horizontal.setPower(0);
                return false;
            }
        }
    }

    public int getPos() {
        return horizontal.getCurrentPosition();
    }
    public int getTimer() {return (int) timer.seconds(); }

    public String getStatus(){
        return status;
    }

    public String getWristStatus(){
        return wristStatus;
    }
    public class SpinnerTime implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < time) {
                spinner.setPower(1);
                return true;
            } else {
                spinner.setPower(0);
                return false;
            }}
    }
    public class WristUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .73;
            wristStatus = "Up";

            wrist.setPosition(targetPos);
            return false;
        }
    }

    public class WristVertical implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .35;
            wristStatus = "Vertical";

            wrist.setPosition(targetPos);
            return false;
        }
    }

    public class WristDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = 0.12;
            wristStatus = "Down";

            wrist.setPosition(targetPos);
            return false;
        }
    }

    public class WristSemi implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.setPosition(.25);
            return false;
        }
    }

    public class WristTravel implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.setPosition(.1);
            return false;
        }
    }

    public class Extend implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            status = "Extended";

            double pos = Math.abs(horizontal.getCurrentPosition());
            if (pos < 1900) {
                horizontal.setTargetPosition(1900);
                horizontal.setPower(.8);
                pos = Math.abs(horizontal.getCurrentPosition());
                return true;
            } else {
                horizontal.setPower(0);
                return false;
            }

        }
    }
    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            status = "Retracted";

            double pos = Math.abs(horizontal.getCurrentPosition());
            if (pos > 400) {
                horizontal.setTargetPosition(400);
                horizontal.setPower(-.9);
                pos = Math.abs(horizontal.getCurrentPosition());
                return true;
            } else {
                horizontal.setPower(0);
                return false;
            }
        }
    }

    public class RetractMid implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = horizontal.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos > 750) {
                horizontal.setTargetPosition(750);
                while( pos > 750){
                    horizontal.setPower(-.5);
                    pos = horizontal.getCurrentPosition();
                }
                horizontal.setPower(0);
                slideExtended = false;
                return true;
            } else {
                return false;
            }
        }
    }

    public class SpinnerIn implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            spinner.setPower(-1);

            return false;
        }
    }
    public class SpinnerOut implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            spinner.setPower(1);

            return false;
        }
    }
    public class SpinnerOff implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            spinner.setPower(0);

            return false;
        }
    }

    public Action spinnerIn(){ return new SpinnerIn(); }
    public Action spinnerOut(){ return new SpinnerOut(); }
    public Action spinnerOff(){ return new SpinnerOff(); }



//    public class Extend implements Action{
//
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            double pos = Math.abs(horizonatal.getCurrentPosition());
//            // 1450 -> counts per rev
//            if (pos < 1500) {
//                horizonatal.setPower(.4);
//                horizonatal.setTargetPosition(1500);
//                while (pos < 1500){
//                    pos = horizonatal.getCurrentPosition();
//                }
//                horizonatal.setPower(0);
//                return false;
//            } else {
//                return false;
//            }
//        }
//    }


}

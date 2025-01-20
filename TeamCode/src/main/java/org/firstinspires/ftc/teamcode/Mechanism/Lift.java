package org.firstinspires.ftc.teamcode.Mechanism;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private DcMotor slides;
    private Servo bucket;
    private boolean init, eaten, slideExtended;

    public Lift(HardwareMap hardwareMap){
        slides = hardwareMap.get(DcMotorEx.class, "lift");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket = hardwareMap.get(Servo.class, "bucket");

        init = true;
        slideExtended = false;
    }

    // public Action retract(){ return new Intake.Retract(); }

    public Action extend(){ return new Lift.Extend(); }
    public Action retract(){ return new Lift.Retract(); }
    public void bucketUp(){
    }
    public Action bucketDown(){ return new Lift.BucketDown(); }
    public class Extend implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = slides.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos < 9700) {
                slides.setTargetPosition(9700);
                while (pos < 9650){
                    slides.setPower(1);
                    slideExtended = true;
                }
                slides.setPower(0);
                return true;
            } else {
                return false;
            }
        }
    }

    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = slides.getCurrentPosition();

            if (pos > 0){
                slides.setTargetPosition(0);
                while (pos > 0){
                    slides.setTargetPosition(9700);
                    slides.setPower(1);
                    slideExtended = true;
                }
                slideExtended = false;
                slides.setPower(0);
                return true;
            }
            return false;
        }
    }

    public class BucketUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.addLine(String.format("CurrentPos%.2f", bucket.getPosition()));
            double targetPos = .95;
            double currentPos = bucket.getPosition();
            while (currentPos < targetPos){
                bucket.setPosition(targetPos - (targetPos - currentPos) / 2);
                currentPos = bucket.getPosition();
                if (targetPos - currentPos <= 0.05){
                    bucket.setPosition(targetPos);
                }
            }
            return true;
        }
    }
    public class BucketDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = 0;
            double currentPos = bucket.getPosition();
            while (currentPos > targetPos){
                bucket.setPosition(currentPos / 2);
                currentPos = bucket.getPosition();
                if (currentPos <= 0.05){
                    bucket.setPosition(targetPos);
                }
            }
            return true;
        }
    }


}

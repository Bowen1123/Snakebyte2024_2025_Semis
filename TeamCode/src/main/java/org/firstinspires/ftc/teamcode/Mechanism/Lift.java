package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    public Servo bucket;
    private boolean init, eaten, slideExtended;
    public Lift(){
        slides = hardwareMap.get(DcMotorEx.class, "lift");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket = hardwareMap.get(Servo.class, "bucket");

        init = true;
        slideExtended = false;
    }

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


    public class Extend implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(slides.getCurrentPosition());
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos < 7500) {
                slides.setTargetPosition(7500);
                while (pos < 7500){
                    slides.setPower(-.8);
                    pos = Math.abs(slides.getCurrentPosition());

                }
                slideExtended = true;
                slides.setPower(0);
                return false;
            } else {
                return false;
            }
        }
    }

    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = slides.getCurrentPosition();

            if (pos > 500){
                slides.setTargetPosition(0);
                while (pos > 500){
                    slides.setTargetPosition(500);
                    slides.setPower(-.8);
                    pos = slides.getCurrentPosition();
                }
                slideExtended = false;
                slides.setPower(0);
                return false;
            }
            if (pos < 500){
                slides.setTargetPosition(500);
                while (pos < 0){
                    slides.setTargetPosition(500);
                    slides.setPower(.8);
                    pos = slides.getCurrentPosition();
                }
                slideExtended = false;
                slides.setPower(0);
                return false;
            }
            return false;
        }
    }

    public class Semi implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(slides.getCurrentPosition());
            // 1450 -> counts per rev
            if (pos < 1000) {
                slides.setTargetPosition(1000);
                while (pos < 1000){
                    slides.setPower(-.4);
                    pos = Math.abs(slides.getCurrentPosition());
                }
                slideExtended = true;
                slides.setPower(0);
                return false;
            } else {
                return false;
            }
        }
    }


    public class BucketUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = 1;

            bucket.setPosition(targetPos);

            return false;
        }
    }
    public class BucketSemi implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .75;
            bucket.setPosition(.75);
            return false;
        }
    }
    public class BucketDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .4;

            bucket.setPosition(targetPos);

            return false;
        }
    }

    public Action extend(){ return new Lift.Extend(); }
    public Action retract(){ return new Lift.Retract(); }
    public Action semiExtend(){ return new Lift.Semi(); }
    public Action bucketUp(){ return new BucketUp(); }
    public Action bucketDown(){ return new Lift.BucketDown(); }
    public Action bucketSemi(){ return new Lift.BucketSemi(); }

}

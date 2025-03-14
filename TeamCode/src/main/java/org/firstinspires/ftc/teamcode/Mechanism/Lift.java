package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public static DcMotor slides;
    public Servo bucket;
    private boolean init, eaten, slideExtended;
    public Lift(){
        slides = hardwareMap.get(DcMotor.class, "lift");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket = hardwareMap.get(Servo.class, "bucket");
    }

    public Lift(HardwareMap hardwareMap){
        slides = hardwareMap.get(DcMotorEx.class, "lift");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket = hardwareMap.get(Servo.class, "bucket");

        init = true;
        slideExtended = false;
    }

    // public Action retract(){ return new Intake.Retract(); }

// I threw a wish in a well
    public class Extend implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(slides.getCurrentPosition());
            if (pos < 8400) { //8450
                slides.setTargetPosition(8400);
                slides.setPower(-1);
                pos = Math.abs(slides.getCurrentPosition());
                return true;
            } else {
                slides.setPower(0);
                return false;
            }
        }
    }

    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(slides.getCurrentPosition());

            if (pos > 750){
                slides.setTargetPosition(750);
                slides.setPower(1);
                pos = Math.abs(slides.getCurrentPosition());
                return true;
            } else {
                slides.setPower(0);
                return false;
            }

        }
    }

    public class Down implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(slides.getCurrentPosition());

            if (pos > 50){
                slides.setTargetPosition(50);
                slides.setPower(1);
                pos = Math.abs(slides.getCurrentPosition());
                return true;
            } else {
                slides.setPower(0);
                return false;
            }

        }
    }
//but now you're in my way
    public class Semi implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(slides.getCurrentPosition());
            if (pos < 750) {
                slides.setTargetPosition(750);
                slides.setPower(-1);
                pos = Math.abs(slides.getCurrentPosition());
                return true;
            } else {
                slides.setPower(0);
                return false;
            }
        }
    }
//hi guys

    public class BucketDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .7;

            bucket.setPosition(targetPos);

            return false;
        }
    }

    public class BucketStart implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .7;

            bucket.setPosition(targetPos);

            return false;
        }
    }

    public class BucketSemi implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .50;
            bucket.setPosition(.50);
            return false;
        }
    }
    public class BucketUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .15;



            bucket.setPosition(targetPos);

            return false;
        }
    }

    public class BucketActivate implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .64;
            bucket.setPosition(targetPos);

            return false;
        }
    }
    public class BucketActivate2 implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .57;

            bucket.setPosition(targetPos);

            return false;
        }
    }

    public Action extend(){ return new Lift.Extend(); }
    public Action retract(){ return new Lift.Retract(); }
    public Action semiExtend(){ return new Lift.Semi(); }
    public Action down() {return new Lift.Down(); }
    public Action bucketUp(){ return new BucketUp(); }
    public Action bucketDown(){ return new Lift.BucketDown(); }
    public Action bucketSemi(){ return new Lift.BucketSemi(); }
    public Action bucketActivate() {return new Lift.BucketActivate();}
    public Action bucketActivate2() {return new Lift.BucketActivate2(); }
    public Action bucketStart(){
        return new Lift.BucketStart();
    }
    public int getPos() {return slides.getCurrentPosition(); }
    public SequentialAction out() {return new SequentialAction(
            extend(),
            bucketSemi(),
            bucketUp(),
            bucketSemi(),
            retract()
    );}

}

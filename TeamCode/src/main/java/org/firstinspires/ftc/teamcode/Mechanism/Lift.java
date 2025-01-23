package org.firstinspires.ftc.teamcode.Mechanism;

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
            double pos = slides.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos < 8525) {
                slides.setTargetPosition(8550);
                while (pos < 8550){
                    slides.setPower(.8);
                }
                slideExtended = true;
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
                    slides.setTargetPosition(0);
                    slides.setPower(.8);
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
            double targetPos = 1;

            bucket.setPosition(targetPos);

            return false;
        }
    }
    public class BucketDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .2;

            bucket.setPosition(targetPos);

            return true;
        }
    }

    public Action extend(){ return new Lift.Extend(); }
    public Action retract(){ return new Lift.Retract(); }
    public Action bucketUp(){ return new BucketUp(); }
    public Action bucketDown(){ return new Lift.BucketDown(); }

}

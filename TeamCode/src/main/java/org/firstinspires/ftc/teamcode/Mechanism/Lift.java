package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public  DcMotor rightLift, leftLift;
    public Servo bucket;
    public int targetPosition;
    private boolean init, eaten, slideExtended;
    private String status = "";
    private String bucketStatus = "";
    public Lift(){
        rightLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket = hardwareMap.get(Servo.class, "bucket");
    }

    public Lift(HardwareMap hardwareMap){
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reverse


        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        bucket = hardwareMap.get(Servo.class, "bucket");

        init = true;
        slideExtended = false;
    }


    public Action goToPos(int targetPosition){
        this.targetPosition = targetPosition;

        return new GoToPos();
    }

    public double getBucketPos(){ return bucket.getPosition(); }
    public double getLiftPos(){ return rightLift.getCurrentPosition(); }
    public String getStatus(){
        return status;
    }

    public String getBucketStatus(){
        return bucketStatus;
    }

    public class GoToPos implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = Math.abs(rightLift.getCurrentPosition());
            if (pos < targetPosition - 5) {
                rightLift.setTargetPosition(targetPosition);
                leftLift.setTargetPosition(targetPosition);
                rightLift.setPower(.8);
                leftLift.setPower(.8);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else if (pos > targetPosition + 5){
                rightLift.setTargetPosition(targetPosition);
                leftLift.setTargetPosition(targetPosition);
                rightLift.setPower(-.8);
                leftLift.setPower(-.8);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else {
                    rightLift.setPower(0);
                    leftLift.setPower(0);
                    return false;
            }
        }
    }

    public class Extend implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            status = "Extended";

            double pos = Math.abs(rightLift.getCurrentPosition());
            if (pos < 2925) { //8450
                rightLift.setTargetPosition(2925);
                leftLift.setTargetPosition(2925);
                rightLift.setPower(1);
                leftLift.setPower(1);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else {
                rightLift.setPower(0);
                leftLift.setPower(0);
                return false;
            }
        }

    }

    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            status = "Retracted";

            double pos = Math.abs(rightLift.getCurrentPosition());

            if (pos > 520) { //8450
                rightLift.setTargetPosition(520);
                leftLift.setTargetPosition(520);
                rightLift.setPower(-.9);
                leftLift.setPower(-.9);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else {
                rightLift.setPower(0);
                leftLift.setPower(0);
                return false;
            }
        }
    }

//but now you're in my way
//hi guys

    public class BucketDown implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .69;
            bucketStatus = "Down";


            bucket.setPosition(targetPos);

            return false;
        }
    }

    public class BucketStart implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .68;
            bucketStatus = "Start";

            bucket.setPosition(targetPos);

            return false;
        }
    }


    public class BucketUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .15;
            bucketStatus = "Up";


            bucket.setPosition(targetPos);

            return false;
        }
    }



    public Action extend(){ return new Lift.Extend(); }
    public Action retract(){ return new Lift.Retract(); }
    public Action bucketUp(){ return new BucketUp(); }
    public Action bucketDown(){ return new Lift.BucketDown(); }
    public Action bucketStart(){
        return new Lift.BucketStart();
    }
    public int getPos() {return rightLift.getCurrentPosition(); }

}

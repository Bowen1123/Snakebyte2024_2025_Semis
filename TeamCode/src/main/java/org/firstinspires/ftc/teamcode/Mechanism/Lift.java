package org.firstinspires.ftc.teamcode.Mechanism;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition_TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Lift {
    public  DcMotor rightLift, leftLift;
    public Servo bucket;
    public int targetPosition;
    private boolean init, eaten, slideExtended;
    private String status = "";
    private String bucketStatus = "";
    public Lift(){
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket = hardwareMap.get(Servo.class, "bucket");
    }

    public Lift(HardwareMap hardwareMap){
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


        bucket = hardwareMap.get(Servo.class, "bucket");

        init = true;
        slideExtended = false;
    }

    public void resetEncoder(){
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setPower(double power){
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
    public Action goToPos(int targetPosition){
        this.targetPosition = targetPosition;

        return new GoToPos();
    }

    public double getBucketPos(){ return bucket.getPosition(); }
    public double getRightLiftPos(){ return rightLift.getCurrentPosition(); }
    public double getLeftLiftPos(){ return leftLift.getCurrentPosition(); }

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
//
//            double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double  turn = gamepad1.right_stick_x;
//            double denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);
//
//            Competition_TeleOp.setDrivePower(
//                    (strafe + linear + turn) / denominator,
//                    (strafe - linear + turn) / denominator,
//                    (strafe - linear - turn) / denominator,
//                    (strafe + linear - turn) / denominator);

            if (pos < targetPosition - 20) {
                rightLift.setTargetPosition(targetPosition);
                leftLift.setTargetPosition(targetPosition);
                rightLift.setPower(.8);
                leftLift.setPower(.8);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else if (pos < targetPosition - 5) {
                rightLift.setTargetPosition(targetPosition);
                leftLift.setTargetPosition(targetPosition);
                rightLift.setPower(.4);
                leftLift.setPower(.4);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else if (pos > targetPosition + 20){
                rightLift.setTargetPosition(targetPosition);
                leftLift.setTargetPosition(targetPosition);
                rightLift.setPower(-.5);
                leftLift.setPower(-.5);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } else if (pos > targetPosition + 5){
                rightLift.setTargetPosition(targetPosition);
                leftLift.setTargetPosition(targetPosition);
                rightLift.setPower(-.25);
                leftLift.setPower(-.25);
                pos = Math.abs(rightLift.getCurrentPosition());
                return true;
            } {
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
            if (pos < 2850) { //2900
                rightLift.setTargetPosition(2850);
                leftLift.setTargetPosition(2850);
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

            if (pos > 200) { //8450
                rightLift.setTargetPosition(200);
                leftLift.setTargetPosition(200);
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
            double targetPos = .62;
            bucketStatus = "Down";


            bucket.setPosition(targetPos);

            return false;
        }
    }

    public class BucketStart implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .615;
            bucketStatus = "Start";

            bucket.setPosition(targetPos);

            return false;
        }
    }


    public class BucketUp implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double targetPos = .13;
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

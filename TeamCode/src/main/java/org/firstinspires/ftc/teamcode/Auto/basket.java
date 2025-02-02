package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism.Lift;

@Autonomous
public class basket extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeM;
    private Servo wrist, bucket;
    private CRServo intake;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(CRServo.class, "spinner");

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
       // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //intakeM = hardwareMap.get(DcMotor.class, "horizonalSlide");


        Lift lift = new Lift(hardwareMap);

        wrist = hardwareMap.get(Servo.class, "wrist");
        bucket = hardwareMap.get(Servo.class, "bucket");
        //leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            int lE = Math.abs(leftBack.getCurrentPosition());
            while (lE  <= 4500 & leftFront.getCurrentPosition()  >= -4500 & rightFront.getCurrentPosition() >= -4500){
                leftBack.setPower(-0.4);
                rightBack.setPower(-0.4);
                leftFront.setPower(-0.4);
                rightFront.setPower(-0.4);
                lE = leftBack.getCurrentPosition();
                telemetry.addData("Pos:   ", lE);
                telemetry.addData("check:   ", leftBack.getCurrentPosition());
                telemetry.addData("lf:   ", leftFront.getCurrentPosition());
                telemetry.addData("rf:   ", rightFront.getCurrentPosition());
                telemetry.update();
            }
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            Actions.runBlocking(new SequentialAction(
                    lift.extend(),
                    lift.bucketUp(),
                    lift.bucketDown(),
                    lift.retract()
                )
            );

            /*ElapsedTime timer = new ElapsedTime();
            timer.startTime();
            timer.reset();

            if (timer.seconds() == 2){
                bucket.setPosition(1);
                telemetry.addData("Bucket Position: ", bucket.getPosition());
                telemetry.update();
            }

            if (timer.seconds() == 6){
                bucket.setPosition(.2);
                telemetry.addData("Bucket Position: ", bucket.getPosition());
                telemetry.update();
            }

            if (timer.seconds() == 10){
                Actions.runBlocking(liftClass.bucketUp());
                telemetry.addData("Bucket Position: ", bucket.getPosition());
                telemetry.update();
            }*/
            //Actions.runBlocking( new SequentialAction(liftClass.bucketUp()));


            /*ElapsedTime elapsedTime = new ElapsedTime();
            while (elapsedTime.seconds() < 2.5){
                leftBack.setPower(-0.5);
                rightBack.setPower(0.5);
                rightFront.setPower(-0.5);
                leftFront.setPower(0.5);
            }
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);*/
        }
    }
}

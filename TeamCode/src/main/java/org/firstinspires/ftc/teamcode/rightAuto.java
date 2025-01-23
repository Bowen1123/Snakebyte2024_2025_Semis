package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanism.Lift;

@Autonomous
public class rightAuto extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeM;
    private DcMotor lift;
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


        //lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
       // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //intakeM = hardwareMap.get(DcMotor.class, "horizonalSlide");


        Lift liftClass = new Lift(hardwareMap);

        wrist = hardwareMap.get(Servo.class, "wrist");
        bucket = hardwareMap.get(Servo.class, "bucket");
        waitForStart();
        while (opModeIsActive()){
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
            Actions.runBlocking( new SequentialAction(liftClass.bucketUp()));


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

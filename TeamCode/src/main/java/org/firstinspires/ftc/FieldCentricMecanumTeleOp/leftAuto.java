package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class leftAuto extends LinearOpMode {
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


        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeM = hardwareMap.get(DcMotor.class, "horizonalSlide");

        wrist = hardwareMap.get(Servo.class, "wrist");
        bucket = hardwareMap.get(Servo.class, "bucket");
        waitForStart();
        bucket.setPosition(.6);
        wrist.setPosition(0.32);
        while (opModeIsActive()){
            int brEnc = rightBack.getCurrentPosition();
            while (brEnc <=16446 ){
                leftFront.setPower(-0.45);
                leftBack.setPower(-0.45);
                rightFront.setPower(-0.35);
                rightBack.setPower(-0.35);
                brEnc = rightBack.getCurrentPosition();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            lift.setTargetPosition(8700);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(-.8);
            ElapsedTime elapsedTime = new ElapsedTime();
            bucket.setPosition(0.6);
            while (elapsedTime.seconds()< 5){
                bucket.setPosition(0.6);
            }
            bucket.setPosition(0.9);
            ElapsedTime e = new ElapsedTime();

            while (e.seconds() < 5){
                bucket.setPosition(0.9);
            }
            terminateOpModeNow();
        }
    }

    public void liftUp(){
        lift.setTargetPosition(8750);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-.8);
    }
}

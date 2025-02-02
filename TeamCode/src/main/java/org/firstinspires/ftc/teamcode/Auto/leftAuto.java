package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

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
        while (opModeIsActive()){

            ElapsedTime time = new ElapsedTime();

            while (time.seconds() < 5){
                leftFront.setPower(.2);
                leftBack.setPower(.2);
                rightFront.setPower(.2);
                rightBack.setPower(.2);
            }

            terminateOpModeNow();
        }
    }

    public void pathing_1(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0 ,0));
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(leftBack.getCurrentPosition(), leftFront.getCurrentPosition(),0 ))
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
    }

    public void liftUp(){
        lift.setTargetPosition(8750);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-.8);
    }
}
// Stuff I moved
/*int brEnc = rightBack.getCurrentPosition();
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

            while (e.seconds()< 5){
                bucket.setPosition(0.9);
            }*/
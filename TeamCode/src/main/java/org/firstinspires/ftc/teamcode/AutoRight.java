package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AutoRight extends LinearOpMode {
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    Servo outtake, intake;
    CRServo eater;
    ElapsedTime timer;
    Encoder par, perp;
    private Pose2d pose;

    IMU imu = hardwareMap.get(IMU.class, "imu");
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        //imu.initialize(parameters);

    //TwoDeadWheelLocalizer localizer = new TwoDeadWheelLocalizer(hardwareMap, 100,1, pose);
    //parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
    //Encoder perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));
    public AutoRight(){
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        outtake = hardwareMap.servo.get("");
        intake = hardwareMap.servo.get("");
        eater = hardwareMap.crservo.get("star");
        timer = new ElapsedTime();
        //par = hardwareMap.
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        //int curloc = localizer.par.getPositionAndVelocity().position;
        int brEncoder = backRightMotor.getCurrentPosition();
        int flEncoder = frontLeftMotor.getCurrentPosition();
        while(opModeIsActive()){
            timer.wait(5);
            while(flEncoder < getTics(2)){
                driveForward(0.5);
                flEncoder = frontLeftMotor.getCurrentPosition();
            }
            stopRobot();
            turnLeft(90);
            while(flEncoder < getTics(37)){
                driveForward(0.5);
                flEncoder = frontLeftMotor.getCurrentPosition();
            }
            stopRobot();
            turnRight(180);
            outtake.setPosition(1);
            timer.wait(1);
            outtake.setPosition(0.5);
            turnRight(180);
            while(flEncoder > getTics(167)){
                driveForward(0.5);
                flEncoder = frontLeftMotor.getCurrentPosition();
            }

        }
    }

    public int getTics(int inches){
        double ticsPerInch = 29.8914660313;
        return (int)(inches * ticsPerInch);
    }

    public void driveForward(double power){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
    }
    public void stopRobot(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }


    public void turnRight(double degrees){

    }
    public void turnLeft(double degrees){

    }


}
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class Teleop extends OpMode {
    IMU imu;
    boolean stopped;

    DcMotor leftFront, leftBack, rightFront, rightBack, lift, horizontalSlide;
    CRServo star;
    Servo outake, intake;

    double turn, strafe,linear, denominator, targetFacing;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;




    @Override
    public void init() {
        // Declare our motors
        // Make sure your ID's match your configuration
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        lift = hardwareMap.dcMotor.get("lift");
        horizontalSlide = hardwareMap.dcMotor.get("horizontalSlide");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        stopped = true;


    }

    @Override
    public void loop() {
        YawPitchRollAngles facing = imu.getRobotYawPitchRollAngles();

        /*if (gamepad1.a){
            outake.setPosition(1);
        }
        if (gamepad1.b){
            outake.setPosition(0);
        }
        if(gamepad1.x){
            outake.setPosition(.5);
        }*/

        if (Math.abs(gamepad2.left_stick_y) > 0.1){
            lift.setPower(gamepad2.left_stick_y);
        }
        else {
            lift.setPower(0);
        }

        if (Math.abs(gamepad2.right_stick_x) > 0.1){
            horizontalSlide.setPower(gamepad2.left_stick_x);
        }
        else {
            horizontalSlide.setPower(0);
        }


        if (gamepad1.right_bumper){
            imuUpdate();
            telemetry.addData("TargetFacing: ", targetFacing);

            telemetry.addData("Stopped: ", stopped);
            telemetry.update();
        }
        if (gamepad1.left_bumper){
            imu.resetYaw();
            targetFacing = 0;
        }

        /*double facingPower = Math.abs(facing.getYaw()) / 30;
        // Left gives positive heading
        if (stopped && facing.getYaw() > 3){
            leftFront.setPower(facingPower);
            leftBack.setPower(facingPower);
            rightFront.setPower(-facingPower);
            rightBack.setPower(-facingPower);
        }
        if (stopped && facing.getYaw() < -3){
            leftFront.setPower(-facingPower);
            leftBack.setPower(-facingPower);
            rightFront.setPower(facingPower);
            rightBack.setPower(facingPower);
        }*/
        strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        turn = gamepad1.right_stick_x;
        denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);
        if (Math.max(Math.abs(strafe) + Math.abs(linear), Math.abs(turn)) > 0.1){

            imu.resetYaw();
        }
        stopped = true;

        double frontLeftPower = (strafe + linear + turn) / denominator;
        double backLeftPower = (strafe - linear + turn) / denominator;
        double frontRightPower = (strafe - linear - turn) / denominator;
        double backRightPower = (strafe + linear - turn) / denominator;
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }


    public void imuUpdate(){
        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        // We need to get the Yaw for Heading
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
    }
}
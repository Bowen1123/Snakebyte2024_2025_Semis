package org.firstinspires.ftc.FieldCentricMecanumTeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
public class Teleop extends LinearOpMode {
    IMU imu;
    boolean stopped;

    double turn, strafe,linear, denominator, targetFacing;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");


        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        stopped = true;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            YawPitchRollAngles facing = imu.getRobotYawPitchRollAngles();

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


            double facingPower = Math.abs(facing.getYaw()) / 30;
            // Left gives positive heading
            if (stopped && facing.getYaw() > 3){
                frontLeftMotor.setPower(facingPower);
                backLeftMotor.setPower(facingPower);
                frontRightMotor.setPower(-facingPower);
                backRightMotor.setPower(-facingPower);
            }
            if (stopped && facing.getYaw() < -3){
                frontLeftMotor.setPower(-facingPower);
                backLeftMotor.setPower(-facingPower);
                frontRightMotor.setPower(facingPower);
                backRightMotor.setPower(facingPower);
            }



            /*strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            turn = gamepad1.right_stick_x;
            denominator = Math.max(Math.abs(strafe) + Math.abs(linear), 1);
            double frontLeftPower = (strafe + linear) / denominator;
            double backLeftPower = (strafe - linear) / denominator;
            double frontRightPower = (strafe - linear) / denominator;
            double backRightPower = (strafe + linear) / denominator;*/

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
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
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
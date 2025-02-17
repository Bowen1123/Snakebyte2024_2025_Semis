package org.firstinspires.ftc.teamcode;

import android.telecom.CallRedirectionService;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricTeleOp extends LinearOpMode{
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeM;
    private DcMotor lift;
    private Servo wrist, bucket;
    private CRServo intake;
    private TouchSensor touchSensor;

    double headingError;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(CRServo.class, "spinner");
        //touchSensor = hardwareMap.touchSensor.get("sensor");


        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeM = hardwareMap.get(DcMotor.class, "horizontal");

        wrist = hardwareMap.get(Servo.class, "wrist");
        bucket = hardwareMap.get(Servo.class, "bucket");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // y-input
            double x = gamepad1.left_stick_x;  // x-input
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if(gamepad1.right_trigger > 0) {
                while (!touchSensor.isPressed()) {
                    intake.setPower(0.7);
                }
                intake.setPower(0);
            }
            if(gamepad1.left_trigger > 0){
                intake.setPower(-0.7);
            } else{
                intake.setPower(0);
            }

            if(gamepad1.dpad_up) {
                lift.setDirection(DcMotorSimple.Direction.REVERSE);
                lift.setPower(0.5);
            }else{
                lift.setPower(0);
            }
            if(gamepad1.dpad_down){
                lift.setDirection(DcMotorSimple.Direction.FORWARD);
                lift.setPower(0.5);
            }else{
                lift.setPower(0);
            }

            if(gamepad1.dpad_left){
                intakeM.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeM.setPower(0.5);
            }else{
                intakeM.setPower(0);
            }

            if(gamepad1.a){
                wrist.setPosition(0);
            }
            if(gamepad1.b){
                wrist.setPosition(1);
            }

            if(gamepad1.x){
                bucket.setPosition(0);
            }
            if(gamepad1.y){
                bucket.setPosition(1);
            }




            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation


            double x_rotation = x * Math.cos(-heading) - y * Math.sin(-heading);
            double y_rotation = x * Math.sin(-heading) + y * Math.cos(-heading);
            
            
            

            x_rotation = x_rotation * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double maxPowerOutput = Math.max(Math.abs(y_rotation) + Math.abs(x_rotation) + Math.abs(rx), 1);
            // Set Motor Powers

            double frontLeftPower = (y_rotation + x_rotation + rx) / maxPowerOutput;
            double backLeftPower = (y_rotation + x_rotation - rx) / maxPowerOutput;
            double frontRightPower = (y_rotation - x_rotation + rx) / maxPowerOutput;
            double backRightPower = (y_rotation - x_rotation - rx) / maxPowerOutput;

            /*
            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;
             */

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }






    }
}
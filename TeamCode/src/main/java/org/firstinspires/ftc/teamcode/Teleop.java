package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

@TeleOp
public class Teleop extends LinearOpMode {
    // Instance Variables
    private IMU imu;
    private Lift lift;
    private Intake intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack, motor, horizontal;
    private double horizontalPower, liftUpPower, liftDownPower;


    @Override
    public void runOpMode() throws InterruptedException {

        map();
        boolean activated = false;
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        CRServo spinner = hardwareMap.get(CRServo.class, "spinner");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        waitForStart();
        while(opModeIsActive()){
            if (!activated){
                // On start, move components to right positions
                // activate(); this was supposed to move the robot to preset position
                activated = true;
            }


            // Retrieve the IMU from the hardware map


            // -------------------- Controls -----------------------

            if (gamepad1.a){
                Actions.runBlocking(new SequentialAction(lift.bucketDown()));
            }
            if(gamepad1.b){
                Actions.runBlocking(new SequentialAction(lift.bucketUp()));
            }

            if (gamepad1.y){
                Actions.runBlocking(new SequentialAction(intake.wristUp()));
            }
            if (gamepad1.x){
                Actions.runBlocking((new SequentialAction(intake.wristDown())));
            }


            // Spinner
            if (gamepad1.right_bumper){
                spinner.setPower(-1);
            } else if (gamepad1.left_bumper) {
                spinner.setPower(1);
            } else {
                spinner.setPower(0);
            }




            liftUpPower = Math.abs(gamepad2.right_stick_y);
            liftDownPower = Math.abs(gamepad2.left_stick_y);


            if (gamepad2.a && motor.getCurrentPosition() < 8350){
                motor.setPower(1); // Up
            } else if (gamepad2.b && motor.getCurrentPosition() > 500){
                motor.setPower(-1);
            } else if (liftUpPower > 0 && motor.getCurrentPosition() < 8350){
                motor.setPower(liftUpPower);
            } else if (liftDownPower > 0 && motor.getCurrentPosition() > 500) {
                motor.setPower(-liftDownPower);
            } else {
                motor.setPower(0);
            }

            motor.setPower(gamepad2.right_stick_y);
            if(gamepad2.dpad_right){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Horizontal
            if (gamepad2.y && horizontal.getCurrentPosition() < 1400){
                horizontal.setPower(1); // Out
            } else if (gamepad2.x && horizontal.getCurrentPosition() > 500){
                horizontal.setPower(-1);
            } else {
                horizontal.setPower(0);
            }


            telemetry.addData("Lift Position: ", motor.getCurrentPosition());
            telemetry.addData("Horizontal Position ", horizontal.getCurrentPosition());


                // -------------------- Drive --------------------------

            double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double  turn = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);

            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;

            leftFront.setPower(frontLeftPower / 1.15);
            leftBack.setPower(backLeftPower / 1.15);
            rightFront.setPower(frontRightPower / 1.15);
            rightBack.setPower(backRightPower / 1.15);
            // -----------------------------------------------------

            telemetry.addData("DPad: ", (gamepad1.dpad_down));
            telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
            telemetry.update();


        }
    }

    // Class Methods
    public void map(){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        motor = hardwareMap.get(DcMotor.class, "lift");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void activate(){
        Actions.runBlocking(new SequentialAction(
                //intake.extend(),
                intake.wristDown(),
                lift.semiExtend(),
                lift.bucketDown(),
                lift.retract()
        ));
    }
    public void transfer(){
        Actions.runBlocking(new SequentialAction(
                lift.bucketDown(),
                lift.retract(),
                intake.wristUp(),
                (new SleepAction(1)),
                intake.wristDown(),
                lift.bucketSemi(),
                lift.extend()
        ));
    }
}

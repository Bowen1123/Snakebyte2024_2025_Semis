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
    private DcMotor leftFront, leftBack, rightFront, rightBack;


    @Override
    public void runOpMode() throws InterruptedException {

        map();
        boolean activated = false;
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        CRServo spinner = hardwareMap.get(CRServo.class, "spinner");
        DcMotor motor = hardwareMap.get(DcMotor.class, "lift");
        TouchSensor sensor = hardwareMap.get(TouchSensor.class, "sensor");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();
        while(opModeIsActive()){
            if (!activated){
                // On start, move components to right positions
                // activate(); this was supposed to move the robot to preset position
                activated = true;
            }


            // Retrieve the IMU from the hardware map

            telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());


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

                if (gamepad2.dpad_up){
                    Actions.runBlocking(new SequentialAction(lift.extend()));
                }  else{
                    motor.setPower(gamepad2.right_stick_y);
                }if (gamepad2.dpad_down){
                    Actions.runBlocking(new SequentialAction(lift.retract()));
                }  else{
                    motor.setPower(gamepad2.right_stick_y);
                }

                if (Math.abs(gamepad1.right_trigger) > 0.1 && gamepad1.right_bumper){
                    spinner.setPower(-gamepad1.right_trigger);
                } else if (Math.abs(gamepad1.right_trigger) > 0.1){
                    spinner.setPower(
                            gamepad1.right_trigger);
                } else {
                    spinner.setPower(0);
                }
                if(gamepad2.dpad_right){
                    wrist.setPosition(0.95);
                }
                if(gamepad2.dpad_left){
                    wrist.setPosition(1);
                }



                // Transfer
                if (gamepad1.left_bumper){
                    transfer();
                }

                if (sensor.isPressed() == true){
                    Actions.runBlocking(new SequentialAction(
                            lift.retract(),
                            lift.bucketDown(),
                            intake.wristUp(),
                            intake.wristDown()
                    ));
                }
                telemetry.addData("Sensor", sensor.isPressed() == true);
                telemetry.update();



                // -------------------- Drive --------------------------
            /*double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = Math.cos(-botHeading) - Math.sin(-botHeading);
            double rotY = Math.sin(-botHeading) + Math.cos(-botHeading);*/

            double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double  turn = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);

            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
            // -----------------------------------------------------


        }
    }

    // Class Methods
    public void map(){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

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

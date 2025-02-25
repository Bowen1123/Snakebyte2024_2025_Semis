package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;
@TeleOp
public class FieldCentricTeleop extends LinearOpMode {

    private IMU imu;
    private Lift lift;
    private Intake intake;
    private CRServo spinner;
    private DcMotor leftFront, leftBack, rightFront, rightBack, motor, horizontal;
    private double horizontalPower, liftUpPower, liftDownPower;


    @Override
    public void runOpMode() throws InterruptedException {
        map();


        waitForStart();
        while (opModeIsActive()){
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


            // Lift
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

            if (gamepad2.right_bumper) {
                Pose2d bucketPose = new Pose2d(5, 77, Math.toRadians(45));
                MecanumDrive drive = new MecanumDrive(hardwareMap, bucketPose);
                //Drive drive =
            }
            if(gamepad2.left_bumper){
                //Pose2d currentPose = drive.getPoseEstimate();

            }


            // Horizontal
            if (gamepad2.y && horizontal.getCurrentPosition() < 2300){
                horizontal.setPower(1); // Out
            } else if (gamepad2.x && horizontal.getCurrentPosition() > 400){
                horizontal.setPower(-1);
            } else {
                horizontal.setPower(0);
            }


            // --------------------------- DRIVE -------------------------------- //

            double y = -gamepad1.left_stick_y; // y-input
            double x = gamepad1.left_stick_x;  // x-input
            double rx = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x_rotation = x * Math.cos(-heading) - y * Math.sin(-heading);
            double y_rotation = x * Math.sin(-heading) + y * Math.cos(-heading);
            x_rotation = x_rotation * 1.1;  // Counteract imperfect strafing
            double maxPowerOutput = Math.max(Math.abs(y_rotation) + Math.abs(x_rotation) + Math.abs(rx), 1);
            double rfPower = (y_rotation - x_rotation - rx) / maxPowerOutput;
            double lfPower = (y_rotation + x_rotation + rx) / maxPowerOutput;
            double lbPower = (y_rotation - x_rotation + rx) / maxPowerOutput;
            double rbPower = (y_rotation + x_rotation - rx) / maxPowerOutput;

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("Lift Position: ", motor.getCurrentPosition());
            telemetry.addData("Horizontal Position ", horizontal.getCurrentPosition());
            telemetry.update();
        }
    }

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

        spinner = hardwareMap.get(CRServo.class, "spinner");


    }

}

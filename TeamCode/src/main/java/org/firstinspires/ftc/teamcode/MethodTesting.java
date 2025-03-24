package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
public class MethodTesting extends LinearOpMode {
    // Instance Variables
    private IMU imu;
    private CRServo spinner;
    private DcMotor leftFront, leftBack, rightFront, rightBack, horizontal, leftLift, rightLift;


    @Override
    public void runOpMode() throws InterruptedException {

        boolean activated = false;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        spinner = hardwareMap.get(CRServo.class, "spinner");

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        Pose2d bucketPose = new Pose2d(7, 78, Math.toRadians(-45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, bucketPose);
        drive.localizer.update();

        Pose2d submersible = new Pose2d(40, -60, Math.toRadians(-90));

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(40, -75), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40, -60), Math.toRadians(-90));


        TrajectoryActionBuilder toBucket = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(40,-75), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(7, 78), Math.toRadians(45));


        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_right){
                drive.localizer.setPose(new Pose2d(7, 78, Math.toRadians(45)));
            }
            telemetry.addData("Current Pose", drive.localizer.getPose());
            if (gamepad1.a){
                Actions.runBlocking(toSubmersible.build());
            }
            if (gamepad1.b){
                Actions.runBlocking(toBucket.build());
            }

            drive.localizer.update();
            // CONTROLS
            /*if (gamepad1.a){
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
            }*/


            SequentialAction firstAuto = new SequentialAction(
                    lift.bucketStart(),
                    lift.extend(),
                    lift.bucketUp(),
                    new SleepAction(.6),
                    lift.bucketDown(),
                    new SleepAction(.6),
                    new ParallelAction(
                            intake.wristDown(),
                            lift.retract()
                    )
            );


            // IMPORTANT ACTIONS HERE
            SequentialAction getBlock = new SequentialAction(
                    intake.wristDown(),
                    intake.activateSpinner(),
                    intake.extend(),
                    new SleepAction(1.9),
                    intake.retract(),
                    intake.deactivateSpinner()
            );

            SequentialAction transferPos = new SequentialAction(
                    lift.goToPos(260),
                    lift.bucketDown(),
                    new SleepAction(.6),
                    intake.goToPos(400),
                    intake.wristUp(),
                    new SleepAction(1),
                    intake.wristDown()
            );



            SequentialAction bucketPos = new SequentialAction(
                    lift.extend(),
                    new SleepAction(.1),
                    lift.bucketUp(),
                    new SleepAction(.6),
                    intake.goToPos(400)
            );

            /*if (gamepad1.b) {
                Actions.runBlocking(getBlock);
            }
            if (gamepad1.a) {
                Actions.runBlocking(firstAuto);
            }
            if (gamepad1.y){
                Actions.runBlocking(transferPos);
            }
            if (gamepad1.x){
                Actions.runBlocking(bucketPos);
            }

            if (gamepad1.y){
                if (!lift.getBucketStatus().equals("Down")){
                    Actions.runBlocking(new SleepAction(.3));
                    Actions.runBlocking(lift.bucketDown());
                }

                if (!lift.getStatus().equals("Retracted")){
                    Actions.runBlocking(lift.retract());
                }

                if (!intake.getStatus().equals("Retracted")){
                    Actions.runBlocking(intake.retract());
                }

                if (!intake.getWristStatus().equals("Up")){
                    Actions.runBlocking(new SleepAction(.3));
                    Actions.runBlocking(intake.wristUp());
                }
            }*/

            // DRIVE
            /*double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
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
            rightBack.setPower(backRightPower / 1.15);*/
            telemetry.update();


        }
    }


}

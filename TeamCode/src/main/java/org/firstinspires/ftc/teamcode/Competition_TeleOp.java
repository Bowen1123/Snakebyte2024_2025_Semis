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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;
@TeleOp
public class Competition_TeleOp extends LinearOpMode {

    private IMU imu;
    private Lift lift;
    private Intake intake;
    private CRServo spinner;
    private DcMotor leftFront, leftBack, rightFront, rightBack, leftLift, rightLift, horizontal;
    private double horizontalPower, liftUpPower, liftDownPower;
    private int horizontal_target_position;
    private  MecanumDrive drives;

    @Override
    public void runOpMode() throws InterruptedException {
        map();

        SequentialAction goToTransfer = new SequentialAction(
                lift.bucketDown(),
                lift.goToPos(180),
                intake.goToPos(200),
                new SleepAction(.5),
                intake.wristUp()
        );


        SequentialAction goToScore = new SequentialAction(
                lift.goToPos(2800),
                new SleepAction(.8),
                lift.bucketUp(),
                new SleepAction(.8),
                lift.bucketDown(),
                lift.goToPos(180)
        );
        TrajectoryActionBuilder toSub = drives.actionBuilder(drives.localizer.getPose())
                .splineTo(new Vector2d(7, 78), Math.toRadians(45));
        TrajectoryActionBuilder toBucket = drives.actionBuilder(drives.localizer.getPose())
                .setTangent(0)
                .splineTo(new Vector2d(40, 78), Math.toRadians(-90));


        waitForStart();
        while (opModeIsActive()) {
            // -------------------- Controls -----------------------

            if (gamepad1.right_trigger > .4) {
                lift.resetEncoder();
                intake.resetEncoder();
                horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontal_target_position = horizontal.getCurrentPosition();
            }

            if (gamepad1.a) {
                Actions.runBlocking(new SequentialAction(
                        lift.bucketDown(),
                        lift.goToPos(180),
                        intake.goToPos(200),
                        new SleepAction(.5),
                        intake.wristUp(),
                        new SleepAction(1.7),
                        intake.wristDown()
                ));
            }
            if (gamepad1.b) {
                Actions.runBlocking(new SequentialAction(
                        lift.goToPos(2800),
                        lift.bucketUp()));

                Actions.runBlocking(new SequentialAction(
                        new SleepAction(2),
                        lift.bucketUp()));

                Actions.runBlocking(new SequentialAction(
                        lift.bucketDown(),
                        lift.goToPos(180)));
            }

            if (gamepad1.y) {
                Actions.runBlocking(new SequentialAction(intake.wristUp()));
            }
            if (gamepad1.x) {
                Actions.runBlocking((new SequentialAction(intake.wristDown())));
            }

            if (gamepad1.dpad_up) {
                Actions.runBlocking(lift.goToPos(1000));
            }
            if (gamepad1.dpad_down) {
                Actions.runBlocking(lift.goToPos(100));
            }
            if (gamepad1.dpad_right) {
                horizontal_target_position -= 2;
                Actions.runBlocking(intake.goToPos(horizontal_target_position));
            }
            if (gamepad1.dpad_left){
                horizontal_target_position += 2;
                Actions.runBlocking(intake.goToPos(horizontal_target_position));
            }



            // Spinner
            if (gamepad1.right_bumper){
                spinner.setPower(-1);
            } else if (gamepad1.left_bumper) {
                spinner.setPower(1);
            } else {
                spinner.setPower(0);
            }




            if (gamepad1.dpad_up){
                // Actions.runBlocking(bucketPos);
            }

            if (gamepad1.dpad_right || gamepad1.dpad_left){
                // Actions.runBlocking(transferPos);
            }



            if (gamepad2.dpad_right && horizontal.getCurrentPosition() < 1950){
                horizontal.setPower(1); // Out
            } else if (gamepad2.dpad_left && horizontal.getCurrentPosition() > 520){
                horizontal.setPower(-1); // In
            } else if (gamepad2.right_stick_x > 0.2 && leftLift.getCurrentPosition() < 1950){
                horizontal.setPower(gamepad2.right_stick_x);
            } else if (gamepad2.right_stick_x < -0.2 && leftLift.getCurrentPosition() > 520){
                horizontal.setPower(gamepad2.right_stick_x);
            } else {
                horizontal.setPower(0);
            }
            horizontal.setPower(gamepad2.right_stick_y);



            if (gamepad2.a){
                Actions.runBlocking(new SequentialAction(lift.goToPos(400)));
            }
            if(gamepad2.b){
                Actions.runBlocking(new SequentialAction(toSub.build()));
            }

            if (gamepad2.y){
                Actions.runBlocking(new SequentialAction(toBucket.build()));
            }
            if (gamepad2.x){
                Actions.runBlocking((new SequentialAction(lift.goToPos(2925))));
            }


            // --------------------------- DRIVE -------------------------------- //

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

            telemetry.addData("D-Pad Up:", gamepad1.dpad_up);
            telemetry.addData("D-Pad Down:", gamepad1.dpad_down);
            telemetry.addData("D-Pad Right:", gamepad1.dpad_right);
            telemetry.addData("D-Pad Left:", gamepad1.dpad_left);

            telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("Lift Position ", lift.getPos());

            telemetry.addData("Horizontal Position ", intake.getPos());
            telemetry.update();
        }
    }




    public void map(){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);
        Pose2d pose2d = new Pose2d(new Vector2d(7, 78), Math.toRadians(45));
        drives = new MecanumDrive(hardwareMap, pose2d);


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);



        spinner = hardwareMap.get(CRServo.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);


    }

}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

//     ./adb connect 192.168.43.1
//     ./adb disconnect 192.168.43.1
@TeleOp
public class Testing extends LinearOpMode {


    private CRServo spinner;
    private DcMotor leftFront, leftBack, rightFront, rightBack, horizontal;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
          //spinner = hardwareMap.get(CRServo.class, "spinner");
//        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
//        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d testPose = new Pose2d(10, 40, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();


        TrajectoryActionBuilder start = drive.actionBuilder(testPose)
                .setTangent(Math.toRadians(0))
//                .strafeTo(new Vector2d(16,38))
                .splineToLinearHeading(new Pose2d(16,40, Math.toRadians(-50)), Math.toRadians(-50))
                .splineTo(new Vector2d(14,38), Math.toRadians(-50)) // 11, 41, -50
                .strafeTo(new Vector2d(10, 42));

        TrajectoryActionBuilder cycle1 = drive.actionBuilder((new Pose2d(10, 42, Math.toRadians(-50))))
                .splineTo(new Vector2d(14, 35), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(17.5,30,0), Math.toRadians(0));

        TrajectoryActionBuilder forward = drive.actionBuilder((new Pose2d(17.5, 30, Math.toRadians(0))))
                .setTangent(Math.toRadians(0))
                .lineToX(21);

        TrajectoryActionBuilder backToBucket = drive.actionBuilder(new Pose2d(22,30,0))
                .splineToLinearHeading(new Pose2d(10,42,-45), Math.toRadians(-5))
                /*
                .splineTo(new Vector2d(33,48), Math.toRadians(-45))
                .waitSeconds(2)
                .splineTo(new Vector2d(30,40), Math.toRadians(-130))
                .waitSeconds(2)
                .splineTo(new Vector2d(13, 46), Math.toRadians(-60))
                .waitSeconds(2)
                .splineTo(new Vector2d(30, 50), Math.toRadians(-180))
                .waitSeconds(2)
                .splineTo(new Vector2d(13, 46), Math.toRadians(-60))*/
                ;


        while(opModeIsActive()){
            if (gamepad1.left_bumper){
                Lift.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Intake.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
//
//            if (gamepad2.y){
//                horizontal.setPower(.4);
//            } else if (gamepad2.x){
//                horizontal.setPower(-.4);
//            } else {
//                horizontal.setPower(0);
//            }


            if (gamepad1.a){


                Actions.runBlocking(new SequentialAction(
//                        lift.bucketActivate(),
//                        new SleepAction(1.5),
//                        lift.bucketActivate2(),
//                        new SleepAction(1.5),
                        lift.bucketSemi(),
                        new SleepAction(.6),
                        intake.wristSemi(),
                        new SleepAction(.5),
                        start.build(),
                        lift.extend(),
                        new SleepAction(.5),
                        lift.bucketUp(),
                        new SleepAction(.4),
                        lift.bucketSemi(),
                        intake.wristTravel(),
                        lift.retract(),
                        new SleepAction(.1),
                        intake.extend(),
                        lift.bucketDown(),
                        new SleepAction(.1)
                ));
                Actions.runBlocking(new SequentialAction(
                        cycle1.build(),
                        intake.wristDown(),
                        new ParallelAction(
                                spinnerTime(2.5, intake),
                                forward.build()
                        ),
                        intake.retractMid(),
                        intake.wristUp(),
                        new SleepAction(.4),
                        intake.wristSemi(),
                        new SleepAction(.4),
                        lift.bucketSemi(),
                        new SleepAction(.5),
                        backToBucket.build(),
                        lift.extend(),
                        lift.bucketUp()
                ));
                sleep(10);
            }

            if (gamepad1.b){
                Actions.runBlocking(new ParallelAction(
                        lift.extend(),
                        lift.retract()
                ));
            }

            if (gamepad1.x){

            }
            if(gamepad1.y){
                telemetry.addData("Clicked", "y");
                telemetry.update();
                Actions.runBlocking(new SequentialAction(
                        intake.extend(),
                        new SleepAction(1),
                        intake.retract()
                ));
                spinnerTime(5, intake);
            }


            double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double  turn = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);

            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;

            leftFront.setPower(frontLeftPower / 2);
            leftBack.setPower(backLeftPower     / 2);
            rightFront.setPower(frontRightPower / 2);
            rightBack.setPower(backRightPower / 2);
            telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw());

            telemetry.addData("Horizontal: ", intake.getPos());
            telemetry.update();


        }

    }
    public SequentialAction spinnerTime(double timer, Intake intake){
        return new SequentialAction(
                intake.spinnerIn(),
                new SleepAction(timer),
                intake.spinnerOff()
        );
    }



}

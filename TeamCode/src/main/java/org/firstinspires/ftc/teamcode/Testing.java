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

        Pose2d startPose = new Pose2d(0, 60, Math.toRadians(-90));
        Pose2d bucketPose = new Pose2d(5, 77, Math.toRadians(45));

        Pose2d endOfOne = new Pose2d(37.5, 50, Math.toRadians(90));

        Pose2d endAtBucket1 = new Pose2d(7,75,Math.toRadians(-45));

        Pose2d endOfTwo = new Pose2d(37.5, 60, Math.toRadians(90));

        Pose2d endAtBucket2 = new Pose2d(7,75,Math.toRadians(-45));

        Pose2d testing = new Pose2d(10, 60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();


        TrajectoryActionBuilder bucket = drive.actionBuilder(startPose)
                .setTangent(0)
                .splineTo(new Vector2d(5, 77), Math.toRadians(45));

        TrajectoryActionBuilder goBackToBucketFromOne = drive.actionBuilder(endOfOne)
                .setTangent(180)
                .splineToLinearHeading(endAtBucket1, Math.toRadians(-45));

        TrajectoryActionBuilder goBackToBucketFromTwo = drive.actionBuilder(endOfTwo)
                .setTangent(180)
                .splineToLinearHeading(endAtBucket2, Math.toRadians(-45));



        TrajectoryActionBuilder one = drive.actionBuilder(bucketPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(20, 50, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(37.5, 50), Math.toRadians(90));
                //.strafeTo(new Vector2d(37.5, 52));

        TrajectoryActionBuilder two = drive.actionBuilder(endAtBucket1)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(25, 60, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(37.5, 60));


        while(opModeIsActive()){
            if (gamepad1.left_bumper){
                Lift.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Intake.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // FIRST ACTION
            if (gamepad1.a){
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                bucket.build(),
                                lift.extend(),
                                intake.spinnerTime(1),
                                lift.bucketActivate()
                        ),
                        lift.bucketUp()
                        //intake.wristTravel()
                ));
            }

            // SECOND ACTION
            if (gamepad1.b){
                Actions.runBlocking(new SequentialAction(
                        lift.bucketDown(),
                        new ParallelAction(
                                intake.wristVertical(),
                                one.build(),
                                lift.retract()
                        ),
                        new ParallelAction(
                                intake.wristDown(),
                                intake.extend(),
                                intake.spinnerTime(3)
                        ),
                        new SleepAction(.3),
                        intake.retract(),
                        new SleepAction(.5),
                        intake.wristUp(),
                        new SleepAction(.7),
                        intake.wristVertical()
                ));
            }

            // THIRD ACTION
            if(gamepad1.y){
                Actions.runBlocking(new SequentialAction(

                        new ParallelAction(
                                goBackToBucketFromOne.build(),
                                lift.extend()
                        ),
                        lift.bucketUp()
                ));
            }

            // FORTH ACTION
            if (gamepad1.x){
                Actions.runBlocking(new SequentialAction(
                        lift.bucketDown(),
                        new ParallelAction(
                                two.build(),
                                lift.retract()
                        ),
                        new ParallelAction(
                                intake.wristDown(),
                                intake.extend(),
                                intake.spinnerTime(3)
                        ),
                        new SleepAction(.3),
                        intake.retract(),
                        new SleepAction(.5),
                        intake.wristUp(),
                        new SleepAction(.7),
                        intake.wristVertical()
                 ));
            }

            // FIFTH ACTION
            if (gamepad1.right_bumper){
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                goBackToBucketFromTwo.build(),
                                lift.extend()
                        ),
                        lift.bucketUp()
                ));
            }


            if (gamepad2.a){
                Actions.runBlocking(new SequentialAction(
                        lift.down()
                ));
            }

            if (gamepad2.b){
                Actions.runBlocking(new SequentialAction(
                        intake.retract()
                ));
            }

            if (gamepad2.x){
                Actions.runBlocking(new SequentialAction(
                        lift.semiExtend(),
                        lift.bucketDown(),
                        new SleepAction(1),
                        intake.wristUp(),
                        new SleepAction(1)
                ));
            }

            if (gamepad2.y){
                Actions.runBlocking(new SequentialAction(
                        intake.wristDown(),
                        intake.extend(),
                        intake.spinnerTime(2),
                        intake.wristTravel(),
                        intake.retract()
                ));
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
            telemetry.addData("Lift: ", lift.getPos());
            telemetry.addData("Timer: ", intake.getTimer());
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

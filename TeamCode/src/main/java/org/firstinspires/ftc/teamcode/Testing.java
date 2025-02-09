package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

//     ./adb connect 192.168.43.1
//     ./adb disconnect 192.168.43.1
@TeleOp
public class Testing extends LinearOpMode {



    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d testPose = new Pose2d(10, 40, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        int vis = 1;
        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(0,40))
                .setTangent(Math.toRadians(-90))
                .lineToY(40);
                //.strafeTo();


        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .strafeTo(new Vector2d(40,40));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(10, 10), Math.toRadians(90));

        TrajectoryActionBuilder start = drive.actionBuilder(testPose)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                //.setTangent(Math.toRadians())
                //.strafeTo(new Vector2d(20, 45))
                .splineTo(new Vector2d(13,46), Math.toRadians(-60));

        while(opModeIsActive()){




            if (gamepad1.a){


                Actions.runBlocking(new SequentialAction(
                        start.build()
                ));
            }

            if (gamepad1.b){

                Actions.runBlocking(new SequentialAction(
                        tab2.build()
                ));
            }

            if (gamepad1.x){
                Actions.runBlocking(new SequentialAction(
                        tab3.build()
                ));
                wait(2);
            }
            if(gamepad1.dpad_down){
                start.build();
            }
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
            telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.update();
        }


        //here is where we would call the methods such as lift.bucketUp to score
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(-10, Math.toRadians(100));




    }
}

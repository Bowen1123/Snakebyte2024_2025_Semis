package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;
@Autonomous
public class BasketAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d testPose = new Pose2d(10, 55, Math.toRadians(0));
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        // TouchSensor sensor = hardwareMap.get(TouchSensor.class,"sensor");

        waitForStart();

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();


        TrajectoryActionBuilder start = drive.actionBuilder(testPose)
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(10, 42), Math.toRadians(-45));

        while (opModeIsActive()){
            Actions.runBlocking(start.build());



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

//        TrajectoryActionBuilder cycle1 = drive.actionBuilder((new Pose2d(7, 42, Math.toRadians(-50))))
//                .splineTo(new Vector2d(12, 36), Math.toRadians(90));
//        //.splineToLinearHeading(new Pose2d(17.5,34,0), Math.toRadians(-10));
//
//        TrajectoryActionBuilder forward = drive.actionBuilder((new Pose2d(17.5, 34, Math.toRadians(-10))))
//                .setTangent(Math.toRadians(0))
//                .lineToX(21.5);
//
//        TrajectoryActionBuilder backToBucket = drive.actionBuilder(new Pose2d(23,30,0))
//                .splineToLinearHeading(new Pose2d(10,42,-45), Math.toRadians(-5));
/*Actions.runBlocking(new SequentialAction(
                    /*lift.bucketSemi(),
                    new SleepAction(.6),
                    intake.wristSemi(),
                    new SleepAction(1.5),
                    lift.bucketStart(),
                    new SleepAction(.5),
                    start.build(),
                    lift.extend(),
                    new SleepAction(.7),
                    lift.bucketUp(),
                    new SleepAction(.7),
                    lift.bucketSemi(),
                    intake.wristTravel(),
                    lift.retract(),
                    new SleepAction(.3),
                    intake.extend(),
                    lift.bucketDown(),
                    new SleepAction(.1)*/
     /*   start.build(),
                    new SleepAction(3.5),
                    cycle1.build()
            ));
                    Actions.runBlocking(new SequentialAction(
        cycle1.build(),
                    intake.wristDown(),
                    new ParallelAction(
        spinnerTime(4, intake),
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
                    lift.bucketUp(),
                    new SleepAction(.5)
            ));*/

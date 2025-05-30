package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

@Autonomous(name = "Comp_Auto", group = "Autonomous")
public class Competition_Autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        lift.resetEncoder();
        intake.resetEncoder();

        Pose2d startPose = new Pose2d(0, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Pose2d bucketPose = new Pose2d(8, 78, Math.toRadians(-45));
        Pose2d nbucketPose = new Pose2d(10, 78, Math.toRadians(-45));

        Pose2d one = new Pose2d(21.5, 70.5, 0);

        Pose2d two = new Pose2d(21, 81, 0);

        Pose2d three = new Pose2d(21, 82, Math.toRadians(45));


        TrajectoryActionBuilder bucket = drive.actionBuilder(startPose)
                .setTangent(0)
                .splineTo(new Vector2d(8, 78), Math.toRadians(45));

        TrajectoryActionBuilder toOne = drive.actionBuilder(bucketPose)
                .setTangent(Math.toRadians(-46))
//                .splineToLinearHeading(new Pose2d(15, 69, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(21.5, 70.5), Math.toRadians(0));

        TrajectoryActionBuilder backToBucketFromOne = drive.actionBuilder(one)
                .setTangent(-135)
//                .splineToConstantHeading(new Vector2d(15, 75), Math.toRadians(-45))
                .splineTo(new Vector2d(10, 78), Math.toRadians(135));


        TrajectoryActionBuilder toTwo = drive.actionBuilder(nbucketPose)
                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(15, 76, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(21, 81), Math.toRadians(45));

        TrajectoryActionBuilder backToBucketFromTwo = drive.actionBuilder(two)
                .setTangent(Math.toRadians(-180))
//                .splineToLinearHeading(new Pose2d(15, 74, Math.toRadians(0)), Math.toRadians(-180))
                .splineTo(new Vector2d(10, 77), Math.toRadians(135));

        TrajectoryActionBuilder toThree = drive.actionBuilder(nbucketPose)
                .setTangent(0)
                .splineTo(new Vector2d(24, 82), Math.toRadians(75));//65

        TrajectoryActionBuilder backToBucketFromThree = drive.actionBuilder(three)
                .setTangent(-135)
                .splineTo(new Vector2d(10, 78), Math.toRadians(100));//115

        /*SequentialAction pathing = new SequentialAction(
                bucket.build(),
                new SleepAction(1),
                toOne.build(),
                new SleepAction(1),
                backToBucketFromOne.build(),
                new SleepAction(1),
                toTwo.build(),
                new SleepAction(1),
                backToBucketFromTwo.build());*/


        /*SequentialAction actions = new SequentialAction(
                intake.spinner(1),
                lift.bucketStart(),
                new SleepAction(.2),
                lift.extend(),
                new SleepAction(.2),

                new ParallelAction(lift.bucketUp(),
                        intake.wristTravel()),

                new SleepAction(2),

                lift.bucketDown(),
                lift.retract(),

                intake.extend(),
                intake.activateSpinner(),
                new SleepAction(2.8),
                intake.deactivateSpinner(),

                intake.wristUp(),
                new SleepAction(1),
                intake.wristDown(),
                lift.extend(),
                lift.bucketUp(),
                lift.retract(),
                new SleepAction(1000)
        ));*/

        waitForStart();

        while (opModeIsActive()  && !isStopRequested()) {
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            lift.bucketStart(),
                            lift.extend()
                            ,
                            bucket.build()
                    ),
                    lift.bucketUp(),
                    intake.wristDown(),
                    new SleepAction(0.6),
                    lift.bucketDown(),
                    new ParallelAction(
                            lift.retract(),
                            toOne.build()
                    ),

                    // Intake First Sample
                    intake.activateSpinner(),
                    intake.extend(),
                    new SleepAction(1.4),
                    intake.deactivateSpinner(),
                    intake.retract(),
                    intake.wristUp(),
                    new SleepAction(1.2),
                    intake.wristSemi(),
                    new ParallelAction(
                            lift.extend(),
                            backToBucketFromOne.build()
                    ),
                    lift.bucketUp(),
                    new SleepAction(.7),
                    lift.bucketDown(),
                    new ParallelAction(
                            lift.retract(),
                            toTwo.build()
                    ),
                    intake.wristDown(),
                    intake.activateSpinner(),
                    intake.extend(),
                    new SleepAction(1.4),
                    intake.retract(),
                    intake.deactivateSpinner(),
                    new SleepAction(.1),
                    intake.wristUp(),
                    new SleepAction(1.6),
                    intake.wristSemi(),
                    new ParallelAction(
                            lift.extend(),
                            backToBucketFromTwo.build()
                    ),
                    lift.bucketUp(),
                    new SleepAction(.6),
                    lift.bucketDown(),


                    // To Three
                    new ParallelAction(
                            lift.retract(),
                            toThree.build()
                    ),
                    intake.wristDown(),
                    intake.activateSpinner(),
                    intake.extend(),
                    new SleepAction(1.3),
                    intake.retract(),
                    intake.deactivateSpinner(),
                    new SleepAction(.1),
                    intake.wristUp(),
                    new SleepAction(1),
                    intake.wristSemi(),
                    new ParallelAction(
                            lift.extend(),
                            backToBucketFromThree.build()
                    ),
                    lift.bucketUp()
                    ));

            Actions.runBlocking(new SleepAction(1000));
        }
    }
}


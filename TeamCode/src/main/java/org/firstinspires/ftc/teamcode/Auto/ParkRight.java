package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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

@Autonomous(name = "Comp_AutoRight", group = "Autonomous")
public class ParkRight extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        lift.resetEncoder();
        intake.resetEncoder();

        Pose2d startPose = new Pose2d(0, 70, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Pose2d bucketPose = new Pose2d(7, 78, Math.toRadians(-45));

        Pose2d one = new Pose2d(21.5, 71, 0);

        Pose2d two = new Pose2d(21, 81.75, 0);

        Pose2d three = new Pose2d(21, 82, Math.toRadians(45));
        TrajectoryActionBuilder park = drive.actionBuilder(startPose)
                .setTangent(90)
                .strafeTo(new Vector2d(0, 58));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Actions.runBlocking(park.build());
        }

    }
}

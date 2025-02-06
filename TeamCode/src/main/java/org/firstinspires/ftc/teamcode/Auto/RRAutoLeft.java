package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

@Config
@Autonomous(name = "Left RR", group = "Autonomous")
public class RRAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Pose2d initialPose = new Pose2d(-30, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        int vis = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(-60);
        //here is where we would call the methods such as lift.bucketUp to score
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(-10, Math.toRadians(100));

        Actions.runBlocking(new SequentialAction(
                tab1.build(),
                lift.bucketUp()
        ));
    }
}

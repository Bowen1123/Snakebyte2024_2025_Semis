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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

@Config
@Autonomous(name = "Left RR", group = "Autonomous")
public class RRAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Pose2d initialPose = new Pose2d(-60, 20, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        int vis = 1;
        Pose2d testPose = new Pose2d(10, 40, Math.toRadians(0));
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(-90))
                .turn(-90)
                .lineToY(40)
                .strafeTo(new Vector2d(-20,50));
        TrajectoryActionBuilder start = drive.actionBuilder(testPose)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                //.setTangent(Math.toRadians())
                //.strafeTo(new Vector2d(20, 45))
                .splineTo(new Vector2d(13,46), Math.toRadians(-60))
                .waitSeconds(2)
                .splineTo(new Vector2d(30,40), Math.toRadians(-130))
                .waitSeconds(2)
                .splineTo(new Vector2d(13, 46), Math.toRadians(-60))
                .waitSeconds(2)
                .splineTo(new Vector2d(30, 50), Math.toRadians(-180))
                .waitSeconds(2)
                .splineTo(new Vector2d(13, 46), Math.toRadians(-60))
                ;





        Actions.runBlocking(new SequentialAction(
                start.build()
        ));
    }
}

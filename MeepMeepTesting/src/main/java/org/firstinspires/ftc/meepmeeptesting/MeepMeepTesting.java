package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 35, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-30, -60, Math.toRadians(180)))
                        .lineTo(new Vector2d(- 52, 60))
                        .build());
                /*      .setTangent(0)
                        .lineTo(new Vector2d(-50, -60)) // Score pre-loaded at basket
                        .addDisplacementMarker(() -> {

                        })
                        .lineTo(new Vector2d(-34, -40))
                        .splineTo(new Vector2d(-34, -10), Math.toRadians(100))
                        .forward(5) // Pick up first element
                        .addDisplacementMarker(() -> {

                        })
                        .lineTo(new Vector2d(-55, -55)) // Score at basket
                        .addDisplacementMarker(() -> {

                        })
                        //.lineTo(new Vector2d(-40, -40))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(180))
                        .forward(3) // Pick up second element
                        .addDisplacementMarker(() -> {

                        })
                        .lineTo(new Vector2d(-55, -55)) // Score at basket
                        .addDisplacementMarker(() -> {

                        })
                        .splineToConstantHeading(new Vector2d(-56, -12), Math.toRadians(180))
                        .forward(3) // Pick up second element
                        .addDisplacementMarker(() -> {

                        })
                        .lineTo(new Vector2d(-55, -55)) // Score at basket
                        .addDisplacementMarker(() -> {

                        })

                        .build());
        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(25, 25, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(30, 60, Math.toRadians(0)))
                        .lineTo(new Vector2d(33, 60))
                        .build());
        RoadRunnerBotEntity extra1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(25, 25, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-30, 60, Math.toRadians(0)))
                        .lineTo(new Vector2d(33, 60))
                        .build());
        RoadRunnerBotEntity extra2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(25, 25, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(30, -60, Math.toRadians(0)))
                        .lineTo(new Vector2d(33, 60))
                        .build());*/


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightBot)
                .start();
    }

    public Vector2d convert(int x, int y){
        return new Vector2d(y, x);
    }

}
package org.firstinspires.ftc.teamcode.byteLibrary;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

public class MecanumDrivePose {
    private final MecanumDriveKinematics kinematics;
    private Pose2d robotPose;
    MecanumDrivePose(MecanumDriveKinematics kinematics, Pose2d startingPose){
        this.kinematics = kinematics;
        this.robotPose = startingPose;
    }
    public void addKinematicsToPose(){
        double[] chassisSpeeds = kinematics.convertWheelSpeedsToChassis(kinematics.getWheelPowers());
        Vector2d transChange = new Vector2d(chassisSpeeds[0],
                                            chassisSpeeds[1]);
        Twist2d changePose = new Twist2d(transChange, chassisSpeeds[1]);
        this.robotPose.plus(changePose);
    }
    public Twist2d toTwist(){
        return this.robotPose.log();
    }
    public void addTwist(Twist2d change){
        this.robotPose.plus(change);
    }
    public void resetPose(Pose2d pose){
        this.robotPose = pose;
    }
    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }
    public Pose2d getRobotPose() {
        return robotPose;
    }
}

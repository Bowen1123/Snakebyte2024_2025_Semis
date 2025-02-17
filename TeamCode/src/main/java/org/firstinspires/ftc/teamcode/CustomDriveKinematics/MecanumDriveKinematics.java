package org.firstinspires.ftc.teamcode.CustomDriveKinematics;

public class MecanumDriveKinematics {
    private final MecanumWheel[] Drives;
    private final double baseRadius; // distance between wheel and center of bot
    MecanumDriveKinematics(MecanumWheel[] wheels, double baseRadius) {
        this.Drives = wheels;
        this.baseRadius = baseRadius;
    }
    public double[] convertChassisSpeedsToWheels(double vForward, double vStrafe, double vRot){
        // vRot counterclockwise is positive
        double[] wheelSpeeds = new double[4];

        double vfl = vForward - vStrafe - (2*baseRadius*vRot);
        double vbl = vForward + vStrafe - (2*baseRadius*vRot);
        double vbr = vForward - vStrafe + (2*baseRadius*vRot);
        double vfr = vForward + vStrafe + (2*baseRadius*vRot);

        wheelSpeeds[0] = vfl; wheelSpeeds[1] = vbl; wheelSpeeds[2] = vbr; wheelSpeeds[3] = vfr;
        return wheelSpeeds;
    }

    public double[] covertWheelSpeedsToChassis(double[] wheelSpeeds){
        // wheel speeds are vfl, vbl, vbr, vfr in order!!!
        double[] chassisSpeeds = new double[4];

        double vForward = (wheelSpeeds[3] + wheelSpeeds[0] + wheelSpeeds[2] + wheelSpeeds[1]) / 4.0;
        double vStrafe = (wheelSpeeds[1] + wheelSpeeds[3] - wheelSpeeds[0] - wheelSpeeds[2]) / 4.0;
        double vRot = (wheelSpeeds[2] + wheelSpeeds[3] - wheelSpeeds[0] - wheelSpeeds[1]) / (4*2*baseRadius);

        chassisSpeeds[0] = vForward; chassisSpeeds[1] = vStrafe; chassisSpeeds[2] = vRot;

        return chassisSpeeds;
    }
    public MecanumWheel[] getDrives() {
        return Drives;
    }
    public double getBaseRadius() {
        return baseRadius;
    }
}

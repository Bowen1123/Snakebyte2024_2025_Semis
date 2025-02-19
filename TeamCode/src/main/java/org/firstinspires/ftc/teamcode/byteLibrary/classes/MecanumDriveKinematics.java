package org.firstinspires.ftc.teamcode.byteLibrary;

public class MecanumDriveKinematics {
    private final MecanumWheel[] Drives;
    private final double baseRadius; // distance between wheel and center of bot
    MecanumDriveKinematics(MecanumWheel[] wheels, double baseRadius) {
        // wheels should be in order vfl, vbl, vbr, vfr
        this.Drives = wheels;
        this.baseRadius = baseRadius;
    }
    public double[] convertChassisSpeedsToWheels(double[] chassisSpeeds){
        // vRot counterclockwise is positive
        double[] wheelSpeeds = new double[4];

        double vfl = chassisSpeeds[0] - chassisSpeeds[1] - (2*baseRadius*chassisSpeeds[2]);
        double vbl = chassisSpeeds[0] + chassisSpeeds[1] - (2*baseRadius*chassisSpeeds[2]);
        double vbr = chassisSpeeds[0] - chassisSpeeds[1] + (2*baseRadius*chassisSpeeds[2]);
        double vfr = chassisSpeeds[0] + chassisSpeeds[1] + (2*baseRadius*chassisSpeeds[2]);

        wheelSpeeds[0] = vfl; wheelSpeeds[1] = vbl; wheelSpeeds[2] = vbr; wheelSpeeds[3] = vfr;
        return wheelSpeeds;
    }

    public double[] convertWheelSpeedsToChassis(double[] wheelSpeeds){
        // wheel speeds are vfl, vbl, vbr, vfr in order!!!
        double[] chassisSpeeds = new double[4];

        double vForward = (wheelSpeeds[3] + wheelSpeeds[0] + wheelSpeeds[2] + wheelSpeeds[1]) / 4.0;
        double vStrafe = (wheelSpeeds[1] + wheelSpeeds[3] - wheelSpeeds[0] - wheelSpeeds[2]) / 4.0;
        double vRot = (wheelSpeeds[2] + wheelSpeeds[3] - wheelSpeeds[0] - wheelSpeeds[1]) / (4*2*baseRadius);

        chassisSpeeds[0] = vForward; chassisSpeeds[1] = vStrafe; chassisSpeeds[2] = vRot;

        return chassisSpeeds;
    }
    public void commandChassisSpeeds(double[] chassisSpeeds){
        double[] wheelSpeeds = convertChassisSpeedsToWheels(chassisSpeeds);
        for (int i = 0; i < Drives.length; i++){
            Drives[i].setMotorPower(wheelSpeeds[i]);
        }
    }
    public int[] getWheelPositions(){
        int[] positions = new int[4];
        for (int i = 0; i < positions.length; i++){
            positions[i] = Drives[i].getCurrentPosition();
        }
        return positions;
    }
    public double[] getWheelPowers(){
        double[] powers = new double[4];
        for (int i = 0; i < powers.length; i++){
            powers[i] = Drives[i].getMotorPower();
        }
        return powers;
    }
    public MecanumWheel[] getDrives() {
        return Drives;
    }
    public double getBaseRadius() {
        return baseRadius;
    }
}

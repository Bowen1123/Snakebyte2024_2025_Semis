package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricDrive extends LinearOpMode{
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeM;
    private DcMotor lift;
    private Servo wrist, bucket;
    private CRServo intake;
    private TouchSensor touchSensor;

    double headingError;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // reset facing (set heading)
            if (gamepad1.right_bumper) {
                imu.resetYaw();
            }


            // --------------------------- DRIVE -------------------------------- //

            double y = -gamepad1.left_stick_y; // y-input
            double x = gamepad1.left_stick_x;  // x-input
            double rx = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double x_rotation = x * Math.cos(-heading) - y * Math.sin(-heading);
            double y_rotation = x * Math.sin(-heading) + y * Math.cos(-heading);
            x_rotation = x_rotation * 1.1;  // Counteract imperfect strafing
            double maxPowerOutput = Math.max(Math.abs(y_rotation) + Math.abs(x_rotation) + Math.abs(rx), 1);
            double rfPower = (y_rotation - x_rotation - rx) / maxPowerOutput;
            double lfPower = (y_rotation + x_rotation + rx) / maxPowerOutput;
            double lbPower = (y_rotation - x_rotation + rx) / maxPowerOutput;
            double rbPower = (y_rotation + x_rotation - rx) / maxPowerOutput;

            /*
            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;
             */

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.update();

        }






    }
}
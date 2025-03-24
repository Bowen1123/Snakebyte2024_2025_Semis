package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;
@TeleOp
public class FieldCentricTeleop extends LinearOpMode {

    private IMU imu;
    private Lift lift;
    private Intake intake;
    private CRServo spinner;
    private DcMotor leftFront, leftBack, rightFront, rightBack, leftLift, rightLift, horizontal;
    private double horizontalPower, liftUpPower, liftDownPower;


    @Override
    public void runOpMode() throws InterruptedException {
        map();


        SequentialAction transferPos = new SequentialAction(
                lift.goToPos(520),
                lift.bucketDown(),
                new SleepAction(.6),
                intake.goToPos(400),
                intake.wristUp(),
                new SleepAction(1),
                intake.wristDown()
        );


        SequentialAction bucketPos = new SequentialAction(
                lift.extend(),
                new SleepAction(.1),
                lift.bucketUp(),
                new SleepAction(.6),
                lift.bucketDown(),
                intake.goToPos(400)
        );



        waitForStart();
        while (opModeIsActive()){
            // -------------------- Controls -----------------------

            if (gamepad1.right_trigger > .4){
                imu.resetYaw();
            }

            if (gamepad1.a){
                Actions.runBlocking(new SequentialAction(lift.bucketDown()));
            }
            if(gamepad1.b){
                Actions.runBlocking(new SequentialAction(lift.bucketUp()));
            }

            if (gamepad1.y){
                Actions.runBlocking(new SequentialAction(intake.wristUp()));
            }
            if (gamepad1.x){
                Actions.runBlocking((new SequentialAction(intake.wristDown())));
            }
            if (gamepad1.dpad_right){

            }


            // Spinner
            if (gamepad1.right_bumper){
                spinner.setPower(-1);
            } else if (gamepad1.left_bumper) {
                spinner.setPower(1);
            } else {
                spinner.setPower(0);
            }




            if (gamepad1.dpad_up){
                Actions.runBlocking(bucketPos);
            }

            if (gamepad1.dpad_right || gamepad1.dpad_left){
                Actions.runBlocking(transferPos);
            }



            leftLift.setPower(gamepad2.right_stick_y);
            rightLift.setPower(gamepad2.right_stick_y);


            // Lift
//            if (gamepad2.dpad_up && leftLift.getCurrentPosition() < 3000){
//                leftLift.setPower(1);
//                rightLift.setPower(1);// Up
//            } else if (gamepad2.dpad_down && leftLift.getCurrentPosition() > 425){
//                leftLift.setPower(-1);
//                rightLift.setPower(-1);// Down
//            } else if (-gamepad2.right_stick_y > 0.2 &&  leftLift.getCurrentPosition() < 3000){
//                leftLift.setPower(gamepad2.right_stick_y);
//                rightLift.setPower(gamepad2.right_stick_y);
//            } else if (-gamepad2.right_stick_y < -0.2 && leftLift.getCurrentPosition() > 425){
//                leftLift.setPower(-gamepad2.right_stick_y / 1.8);
//                rightLift.setPower(-gamepad2.right_stick_y / 1.8);
//            } else {
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//            }

            if (gamepad2.dpad_right && horizontal.getCurrentPosition() < 1950){
                horizontal.setPower(1); // Out
            } else if (gamepad2.dpad_left && horizontal.getCurrentPosition() > 520){
                horizontal.setPower(-1); // In
            } else if (gamepad2.left_stick_x > 0.2 && leftLift.getCurrentPosition() < 1950){
                horizontal.setPower(gamepad2.right_stick_x);
            } else if (gamepad2.left_stick_x < -0.2 && leftLift.getCurrentPosition() > 520){
                horizontal.setPower(gamepad2.right_stick_x);
            } else {
                horizontal.setPower(0);
            }



            if (gamepad2.a){
                Actions.runBlocking(new SequentialAction(lift.goToPos(400)));
            }
            if(gamepad2.b){
                Actions.runBlocking(new SequentialAction(lift.goToPos(300)));
            }

            if (gamepad2.y){
                Actions.runBlocking(new SequentialAction(lift.goToPos(200)));
            }
            if (gamepad2.x){
                Actions.runBlocking((new SequentialAction(lift.goToPos(2925))));
            }


            // --------------------------- DRIVE -------------------------------- //

            double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double  turn = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);

            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;

            leftFront.setPower(frontLeftPower / 1.15);
            leftBack.setPower(backLeftPower / 1.15);
            rightFront.setPower(frontRightPower / 1.15);
            rightBack.setPower(backRightPower / 1.15);

            telemetry.addData("D-Pad Up:", gamepad1.dpad_up);
            telemetry.addData("D-Pad Down:", gamepad1.dpad_down);
            telemetry.addData("D-Pad Right:", gamepad1.dpad_right);
            telemetry.addData("D-Pad Left:", gamepad1.dpad_left);

            telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("Lift Position: ", rightLift.getCurrentPosition());
            telemetry.addData("Horizontal Position ", horizontal.getCurrentPosition());
            telemetry.update();
        }
    }

    public void map(){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);


        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


        rightLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner = hardwareMap.get(CRServo.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);


    }

}

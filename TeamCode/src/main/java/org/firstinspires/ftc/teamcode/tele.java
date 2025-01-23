package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism.Lift;
@TeleOp
public class tele extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeM;
    private DcMotor lift;
    private Servo wrist, bucket;
    private CRServo intake;
    private boolean init;
    private int LIFT_TARGET_POSITION = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(CRServo.class, "spinner");
        init = false;


        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Lift testingLift = new Lift(hardwareMap);

        //intakeM = hardwareMap.get(DcMotor.class, "horizonalSlide");

        wrist = hardwareMap.get(Servo.class, "wrist");
        bucket = hardwareMap.get(Servo.class, "bucket");

        waitForStart();
        while (opModeIsActive()){
           double strafe = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double linear = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double  turn = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(strafe) + Math.abs(linear) + Math.abs(turn), 1);
            int brEnc= rightBack.getCurrentPosition();
            telemetry.addData("Encoder: ",brEnc);
            double frontLeftPower = (strafe + linear + turn) / denominator;
            double backLeftPower = (strafe - linear + turn) / denominator;
            double frontRightPower = (strafe - linear - turn) / denominator;
            double backRightPower = (strafe + linear - turn) / denominator;
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

             if (gamepad2.right_bumper && init == false){
                 bucket.setPosition(.6);
                 wrist.setPosition(0.32);
                 init = true;
            }
            if(gamepad2.dpad_right){
                leftFront.setPower(1);
            }
            if (gamepad2.a){
                bucket.setPosition(0.2);
            } if (gamepad2.x){
                bucket.setPosition(1);
            }
            if(gamepad2.b){
                //wrist.setPosition(0.32);
                wristDown();
            }
            if (gamepad2.y){
                wrist.setPosition(0.7);
            }

            if (gamepad2.left_bumper){
                testingLift.bucketUp();
            }

            /*if(gamepad2.left_bumper){
                intake.setPower(1);
            } else if (gamepad2.right_bumper){
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }*/

            LIFT_TARGET_POSITION += gamepad2.right_stick_y * 10;
            lift.setTargetPosition(LIFT_TARGET_POSITION);
            if (LIFT_TARGET_POSITION < 0){
                LIFT_TARGET_POSITION = 1;
            }
            if (LIFT_TARGET_POSITION > 8750){
                LIFT_TARGET_POSITION = 8750;
            }
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (LIFT_TARGET_POSITION == 0){
                lift.setPower(LIFT_TARGET_POSITION/ (Math.abs(LIFT_TARGET_POSITION) + 1));
            }
            else {
                lift.setPower(LIFT_TARGET_POSITION/Math.abs(LIFT_TARGET_POSITION));
            }


            telemetry.addData("Lift Position:", lift.getCurrentPosition());
            telemetry.addData("LIFT_TARGET_POSITION:", LIFT_TARGET_POSITION);
            telemetry.update();
        }

    }

    public void liftUp(){
        lift.setTargetPosition(8250);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine();
        telemetry.addData("Lift Status:", 1);
        lift.setPower(-.8);
    }
    public void liftDown(){
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(.8);
    }


    public void wristDown(){
        double targetPos = 0.20;
        for (int i =0; i <= 6; i++){
            if (wrist.getPosition() > .20){
                wrist.setPosition(targetPos + (targetPos - wrist.getPosition()) / 2);
                if (wrist.getPosition() <= .25){
                    wrist.setPosition(targetPos);
                }
            }
        }
    }
}


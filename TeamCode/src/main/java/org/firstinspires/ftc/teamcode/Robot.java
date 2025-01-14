package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeM;
    private DcMotor lift;
    private Servo spinner, bucket;

    public Robot(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeM = hardwareMap.get(DcMotor.class, "horizonalSlide");

        spinner = hardwareMap.get(Servo.class, "spinner");
        bucket = hardwareMap.get(Servo.class, "bucket");
    }

    public boolean slideUp(){
        return true;
    }


    public double distanceToCount(double distance, double radius){
        double counts = (distance / radius) * 1450 ;
        // distance = counts * revolution * radius
        return counts;
    }

}


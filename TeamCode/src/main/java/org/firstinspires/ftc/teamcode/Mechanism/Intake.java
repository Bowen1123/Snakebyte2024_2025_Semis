package org.firstinspires.ftc.teamcode.Mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake{
    private DcMotor slides;
    private CRServo spinner;
    private TouchSensor eater;
    private boolean init, eaten, slideExtended;

    public Intake(HardwareMap hardwareMap){
        slides = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner = hardwareMap.get(CRServo.class, "spinner");
        eater = hardwareMap.get(TouchSensor.class, "eater");

        init = true;
        slideExtended = false;
    }

    public Action retract(){
        return new Retract();
    }
    public Action extend() {return new Extend(); }
    public Action eat() {return new Eat(); }

    public class Retract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = slides.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos < 3000.0) {
                slides.setTargetPosition(3000);
                slideExtended = true;
                return true;
            } else {
                return false;
            }
        }
    }

    public class Extend implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = slides.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            // 1450 -> counts per rev
            if (pos > 0) {
                slides.setTargetPosition(0);
                slideExtended = false;
                return true;
            } else {
                return false;
            }
        }
    }

    public class Eat implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            eaten = eater.isPressed();
            while (eaten != true){
                spinner.setPower(1);
            }
            spinner.setPower(0);
            if (slideExtended){
                retract();
            }
            return true;
        }
    }

    public void throwUp(){

        spinner.setPower(-1);

    }
}

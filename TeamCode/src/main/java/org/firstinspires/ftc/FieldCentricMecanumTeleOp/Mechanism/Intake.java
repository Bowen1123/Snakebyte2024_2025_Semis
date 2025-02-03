package org.firstinspires.ftc.FieldCentricMecanumTeleOp.Mechanism;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake{
    private DcMotor slides;
    private DcMotor Servo;

    public Intake(HardwareMap hardwareMap){
        slides = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Action retract(){
        return new Retract();
    }

    public class Retract implements Action{
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!initialized) {
                initialized = true;
            }

            double pos = slides.getCurrentPosition();
            telemetryPacket.put("liftPos", pos);
            if (pos < 3000.0) {
                return true;
            } else {
                slides.setTargetPosition(3000);
                return false;
            }
        }
    }
}

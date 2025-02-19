package org.firstinspires.ftc.teamcode.byteLibrary.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.byteLibrary.classes.MotorController;

public class MotorControllerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorController controller = new MotorController(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                                                         DcMotor.ZeroPowerBehavior.FLOAT,
                                                         DcMotorSimple.Direction.FORWARD,
                                                         "motor",
                                                         hardwareMap);
        controller.setMotorVelocity(0.8);
        controller.setMotorPosition(100);
    }
}

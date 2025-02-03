package org.firstinspires.ftc.FieldCentricMecanumTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.FieldCentricMecanumTeleOp.Mechanism.Intake;

@Autonomous(name="Testing", group = "Test")
public class Testing extends LinearOpMode {
    private DcMotor lift;

    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
            lift = hardwareMap.get(DcMotor.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            intake = new Intake(hardwareMap);

            //2750
            waitForStart();
        while (opModeIsActive()) {
            intake.retract();


            /*telemetry.addData("Lift Position: ", lift.getCurrentPosition());
            telemetry.update();

            lift.setTargetPosition(1450);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(.5);
            if (lift.getCurrentPosition() == 1000){
                telemetry.addData("Counts: ", 1000);
            }*/
        }
    }



}
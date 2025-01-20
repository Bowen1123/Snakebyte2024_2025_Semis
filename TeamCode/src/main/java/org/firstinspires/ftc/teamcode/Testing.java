package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanism.Intake;
import org.firstinspires.ftc.teamcode.Mechanism.Lift;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Testing extends LinearOpMode {
    private DcMotor lift;
    private FtcDashboard dash = FtcDashboard.getInstance();

    private Intake intake;
    private MecanumDrive drive;
    private List<Action> actions = new ArrayList<>();
    private int maxHeight;

    @Override
    public void runOpMode() throws InterruptedException {
            lift = hardwareMap.get(DcMotor.class, "lift");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            maxHeight = 0;

            Lift liftClass = new Lift(hardwareMap);
            liftClass.bucketDown();

            //1450
            waitForStart();
            while (opModeIsActive()){
                TelemetryPacket packet = new TelemetryPacket();



                if (lift.getCurrentPosition() > maxHeight){
                    maxHeight = lift.getCurrentPosition();
                }

                if (Math.abs(gamepad1.left_stick_y) > 0.1){
                    lift.setPower(gamepad1.left_stick_y);
                } else if (gamepad1.x){
                    lift.setTargetPosition(maxHeight);
                    lift.setPower(1);
                } else {
                    lift.setPower(0);
                }

                if(gamepad1.right_bumper) {
                    lift.setMode(DcMotor.RunMode.RESET_ENCODERS);
                }
                if(gamepad1.a){
                    liftClass.bucketUp();
                    Actions.runBlocking(liftClass.bucketDown());
                    telemetry.addData("done", 0);
                    telemetry.update();
                }



                if (gamepad1.right_bumper){
                    actions.clear();
                }

            /* List<Action> newActions = new ArrayList<>();
            for (Action action : actions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actions = newActions;

            dash.sendTelemetryPacket(packet);
            /*telemetry.addData("Lift Position: ", lift.getCurrentPosition());
            telemetry.update();

            lift.setTargetPosition(1450);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(.5);
            if (lift.getCurrentPosition() == 1000){
                telemetry.addData("Counts: ", 1000);
            }*/
                telemetry.addData("Counts", lift.getCurrentPosition());
                telemetry.update();
        }
    }



}
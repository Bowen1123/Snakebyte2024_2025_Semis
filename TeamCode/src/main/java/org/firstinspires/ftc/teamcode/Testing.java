package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
            //1450
            waitForStart();
            while (opModeIsActive()){
                TelemetryPacket packet = new TelemetryPacket();

                if(gamepad1.a){
                    liftClass.bucketUp();
                    Actions.runBlocking(liftClass.bucketUp());
                    telemetry.addData("done", 0);
                    telemetry.update();
                }


            if (gamepad2.left_bumper){
                actions.add(liftClass.bucketUp());
            }
            if (gamepad2.right_bumper && actions.size() > 0){
                telemetry.addData("Running", 1);
                Actions.runBlocking(actions.get(0));
                actions.remove(0);
            }
            if (gamepad2.a){
                Actions.runBlocking(new InstantAction(() -> liftClass.bucket.setPosition(1)));
            }

            telemetry.addData("Actions Size", actions.size());
            telemetry.update();
        }
    }



}
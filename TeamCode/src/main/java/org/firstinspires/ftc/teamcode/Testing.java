package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

            if (gamepad2.left_bumper){
                actions.add(liftClass.bucketUp());
            }

            if (gamepad2.right_bumper){
                Actions.runBlocking(new SequentialAction(liftClass.bucketUp() ));
            }

            if (gamepad2.b){
                Actions.runBlocking(new ParallelAction(
                        liftClass.bucketDown(),
                        liftClass.extend()
                ));
            }

            telemetry.addData("Actions Size", actions.size());
            telemetry.update();
        }
    }



}
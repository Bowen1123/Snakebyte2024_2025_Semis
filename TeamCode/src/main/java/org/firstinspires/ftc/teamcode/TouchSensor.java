package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TouchSensor {
    public TouchSensor touchSensor;
    public Servo intake;
    public Servo outtake;

    public TouchSensor(HardwareMap hardwareMap) {
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor");
    }


}

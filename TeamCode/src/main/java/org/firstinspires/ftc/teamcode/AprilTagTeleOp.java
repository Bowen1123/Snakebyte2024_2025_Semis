/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagTeleOp extends LinearOpMode {

    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        // This is outline for math in EP, just for pictures

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();


        for (AprilTagDetection detection : currentDetections){
//            detection.ftcPose = new AprilTagPoseFtc();
//
//            int id = detection.id;
//            detection.ftcPose.x =  detection.rawPose.x;
//            detection.ftcPose.y =  detection.rawPose.z;
//            detection.ftcPose.z = -detection.rawPose.y;

        }

    }
}*/
// this doesn't build? you are missing arguments in AprilTagPoseFTC(), you should move detection.rawPose there

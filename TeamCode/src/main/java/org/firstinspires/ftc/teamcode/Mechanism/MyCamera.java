package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

// Used for Object Detection not Apriltags, that's somewhere else


public class MyCamera {
    private OpenCvWebcam camera;
    private OpenCvPipeline pipeline_1;

    public MyCamera(HardwareMap hardwareMap){
        camera = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");
    }

    public void setPipeline_1(){
        camera.setPipeline(pipeline_1);
    }

}



class drawRectangle extends OpenCvPipeline{

    @Override
    public Mat processFrame(Mat input) {



        return null;
    }
}


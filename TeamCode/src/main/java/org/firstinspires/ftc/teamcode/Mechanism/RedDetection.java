package org.firstinspires.ftc.teamcode.Mechanism;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.CvType;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class RedDetection extends OpenCvPipeline {

    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat resultMat = new Mat();
    private String position = "Unknown";

    @Override
    public Mat processFrame(Mat input) {
        int frameWidth = input.width();
        int frameHeight = input.height();

        // Convert to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define HSV range for red color (two ranges for red)
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Threshold the image to get red regions
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Core.inRange(hsvMat, lowerRed1, upperRed1, mask1);
        Core.inRange(hsvMat, lowerRed2, upperRed2, mask2);

        // Combine both masks
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, thresholdMat);

        // Find contours to determine position
        Mat contoursMat = new Mat();
        Imgproc.findContours(thresholdMat, new java.util.ArrayList<>(), contoursMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (!contoursMat.empty()) {
            Rect boundingBox = Imgproc.boundingRect(contoursMat);
            int centerX = boundingBox.x + boundingBox.width / 2;
            int centerY = boundingBox.y + boundingBox.height / 2;

            // Determine position based on 9 sections
            if (centerY < frameHeight / 3) {
                if (centerX < frameWidth / 3) position = "Top Left";
                else if (centerX < 2 * frameWidth / 3) position = "Top Center";
                else position = "Top Right";
            } else if (centerY < 2 * frameHeight / 3) {
                if (centerX < frameWidth / 3) position = "Middle Left";
                else if (centerX < 2 * frameWidth / 3) position = "Middle Center";
                else position = "Middle Right";
            } else {
                if (centerX < frameWidth / 3) position = "Bottom Left";
                else if (centerX < 2 * frameWidth / 3) position = "Bottom Center";
                else position = "Bottom Right";
            }
        }

        // Convert single-channel mask to 3-channel output for viewing
        Imgproc.cvtColor(thresholdMat, resultMat, Imgproc.COLOR_GRAY2RGB);

        // Return the filtered image back to the driver station
        return resultMat;
    }

    public String getPosition() {
        return position;
    }
}

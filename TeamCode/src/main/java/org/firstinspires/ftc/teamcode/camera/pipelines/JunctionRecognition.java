package org.firstinspires.ftc.teamcode.camera.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionRecognition extends OpenCvPipeline {

    // Notice this is declared as an instance variable (and re-used), not a local variable
    Scalar lowYellow = new Scalar(20, 70, 80);
    Scalar highYellow = new Scalar(32, 255, 255);


    @Override
    public Mat processFrame(Mat input) {

        Mat blur = new Mat();
//        Blur the image to reduce noise
        Imgproc.GaussianBlur(input, blur, new Size(7, 7), 4);

        Mat hsv = new Mat();
//        Convert the blurred image to HSV, for easier color recognition
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        Mat thresh = new Mat();
//        Get a b&w image of yellow objects
        Core.inRange(hsv, lowYellow, highYellow, thresh);

        Mat mask = new Mat();
//        Color the white portion of thresh with HSV
        Core.bitwise_and(hsv, hsv, mask, thresh);

//        Calculate average HSV values of the white thresh values
        Scalar average = Core.mean(mask, thresh);

//        Average saturation
        Mat scaledMask = new Mat();
        mask.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Scalar strictLowYellow = new Scalar(0, 110, 60);
        Scalar strictHighYellow = new Scalar(255, 255, 255);

        Mat scaledThresh = new Mat();
        Core.inRange(scaledMask, strictLowYellow, strictHighYellow, scaledThresh);

        Mat finalMask = new Mat();
        Core.bitwise_and(hsv, hsv, finalMask, scaledThresh);

        Mat edges = new Mat();

        Imgproc.Canny(finalMask, edges, 100, 200);


//        Preview image in RGB




        input.release();
//        edges.release();
        scaledThresh.release();
        scaledMask.release();
        thresh.release();
        mask.release();
        blur.release();
        hsv.release();

        return edges;

    }

}

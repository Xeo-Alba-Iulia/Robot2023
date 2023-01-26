package org.firstinspires.ftc.teamcode.camera;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AprilRecognition extends OpenCvPipeline {
  //Scalar blue= new Scalar(4,197,235,255);
  //Scalar red=new Scalar(4,)

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols() / 4.0,
                        input.rows() / 4.0),
                new Point(
                        input.cols() * (3.0 / 4.0),
                        input.rows() * (3.0 / 4.0)),
                new Scalar(65,105,225), 4);

        return input;
    }




}

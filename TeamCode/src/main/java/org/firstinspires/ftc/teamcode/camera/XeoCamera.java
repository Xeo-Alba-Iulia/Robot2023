package org.firstinspires.ftc.teamcode.camera;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class XeoCamera {
    OpenCvCamera camera;
    AprilRecognition aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    final double TAGSIZE = 0.166;

    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;

    int cameraMonitorViewId;
    public XeoCamera() {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam
                (hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilRecognition(TAGSIZE, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("n-o mers camera că", errorCode);
                telemetry.update();

            }
        });
    }

    public int getCase() {

          ArrayList<AprilTagDetection> currentDetection = aprilTagDetectionPipeline.getLatestDetections();
        for(AprilTagDetection tag : currentDetection) {
            if(tag.id == CASE_1 || tag.id == CASE_2 || tag.id == CASE_3) {
                switch (tag.id) {
                    case CASE_2:
                        return CASE_2;
                    case CASE_3:
                        return CASE_3;
                    default:
                        return CASE_1;
                }
            }
        }
        return 0;
    }


}

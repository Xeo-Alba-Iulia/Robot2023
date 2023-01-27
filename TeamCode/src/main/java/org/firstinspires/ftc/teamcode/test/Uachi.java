package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.AprilRecognition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous(name = "ochio", group = "test")
public class Uachi extends LinearOpMode {

//    public static final String VUFORIA_LICENSE_KEY =
    //        "AYfZrN7/////AAABmSI0fXxlq0JUizUttWoT0hcxZ/HbM/9gcOaBvJtQwao0ba1iEivW9sQTl4hRzRZAKt9pMKJ0Hfpae6nS//q41Dclwf60TgdObekG+X5Q4LX/a8fEWglcvrYqM+zYAx6ishks9atTJOtfHhIVB3gzuKW8xh4RF7yjLBWFVRR2b3k510oUSMSkXLT6RaNWcca8Vz3HwPnm+gHThxRYP8HFkcHlAjnMTDQswNMjbVt3xMh/e3yVk+TTRNYHluIs46clTR+hqbNo4BAeXNhHTFgWWPBZbtFxwtyGn/Rlchgd+3S74s6h94ulFTXtQeB0Qb7uvYi5b6lkYqZnflcSonj+ACw2F+bd/XM9DLz8GtjLeBzX";

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

       // WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),cameraMonitorViewId);

        camera.setPipeline(new AprilRecognition(0.036));
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {


            @Override
            public void onOpened() {
                camera.startStreaming(800,448 , OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
                dashboardTelemetry.addData("Eroare", errorCode);
                dashboardTelemetry.update();
            }




        });
        dashboard.startCameraStream(camera, 30);
        waitForStart();
        if(isStopRequested())
            return;
        while(opModeIsActive()){
         //   dashboardTelemetry.addData("fps", camera.getFps());
         //   dashboardTelemetry.update();


        };
        }
    }

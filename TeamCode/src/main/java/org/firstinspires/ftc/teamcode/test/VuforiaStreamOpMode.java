package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous(name = "camera", group = "test")
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY =
            "AYfZrN7/////AAABmSI0fXxlq0JUizUttWoT0hcxZ/HbM/9gcOaBvJtQwao0ba1iEivW9sQTl4hRzRZAKt9pMKJ0Hfpae6nS//q41Dclwf60TgdObekG+X5Q4LX/a8fEWglcvrYqM+zYAx6ishks9atTJOtfHhIVB3gzuKW8xh4RF7yjLBWFVRR2b3k510oUSMSkXLT6RaNWcca8Vz3HwPnm+gHThxRYP8HFkcHlAjnMTDQswNMjbVt3xMh/e3yVk+TTRNYHluIs46clTR+hqbNo4BAeXNhHTFgWWPBZbtFxwtyGn/Rlchgd+3S74s6h94ulFTXtQeB0Qb7uvYi5b6lkYqZnflcSonj+ACw2F+bd/XM9DLz8GtjLeBzX";


    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 15);

        waitForStart();

        while (opModeIsActive());
    }
}
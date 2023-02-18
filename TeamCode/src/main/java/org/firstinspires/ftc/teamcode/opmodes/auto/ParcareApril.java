package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.camera.AprilRecognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "A", preselectTeleOp = "TeleOP")
public class ParcareApril extends OpMode {

    final static double TAGSIZE = 0.166;
    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;

    SampleMecanumDrive drive;
    Pose2d startPose;
    RobotHardware robot;
    TrajectorySequence secventa3;
    TrajectorySequence secventa2;
    TrajectorySequence secventa1;
    OpenCvCamera camera;
    AprilRecognition aprilRecognition;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int cameraMonitorViewid;


    //Trajectory inceput, align, cone, mid;

    @Override
    public void init() {
        cameraMonitorViewid = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam
                (hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewid);
        aprilRecognition = new AprilRecognition(TAGSIZE, fx, fy, cx, cy);
        camera.setPipeline(aprilRecognition);
        camera.setPipeline(aprilRecognition);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("nu merge camera", errorCode);
                telemetry.update();

            }
        });
        robot = new RobotHardware(this);
        startPose = new Pose2d(32, -65.5, Math.toRadians(270.00));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        secventa3 = drive.trajectorySequenceBuilder(new Pose2d(32, -65.5, Math.toRadians(270.00)))
                .back(25.5)
                .strafeLeft(26.5)
                .build();
        secventa1 = drive.trajectorySequenceBuilder(new Pose2d(36, -65.5, Math.toRadians(270)))
                .back(25.5)
                .strafeRight(26.5)
                .build();

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        secventa2 = drive.trajectorySequenceBuilder(new Pose2d(35, -63.5, Math.toRadians(270)))
                .back(30)
                .build();
    }

    @Override
    public void start() {
        ArrayList<AprilTagDetection> currentDetections = aprilRecognition.getLatestDetections();
        if (currentDetections.size() != 0) {

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == CASE_1) drive.followTrajectorySequence(secventa1);
                else if(tag.id == CASE_2) {
                    drive.followTrajectorySequence(secventa2);
                }

                else if (tag.id == CASE_3) drive.followTrajectorySequence(secventa3);
            }
        }
        robot.start();

    }

    @Override
    public void loop() {
        drive.update();
    }
}

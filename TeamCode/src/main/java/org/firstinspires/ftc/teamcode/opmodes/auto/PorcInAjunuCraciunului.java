package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class PorcInAjunuCraciunului extends OpMode {

    SampleMecanumDrive drive;

    Pose2d startPose;
    RobotHardware robot;
    TrajectorySequence secventa3;
    TrajectorySequence secventa2;
    TrajectorySequence secventa1;

    TrajectorySequence secventa_con1;
    TrajectorySequence secventa_con2;
    TrajectorySequence secventa_con3;

    OpenCvCamera camera;
    AprilRecognition aprilRecognition;
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    final double TAGSIZE = 0.166;

    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;


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
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("nu merge camera", errorCode);
                telemetry.update();

            }
        });
        robot = new RobotHardware(this);
        startPose = new Pose2d(35, -60.355, Math.toRadians(270));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(32, -65.5, Math.toRadians(270.00)));
        secventa3 = drive.trajectorySequenceBuilder(new Pose2d(32, -65.5, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(55, -40.50), 0)
                .build();


        secventa1 = drive.trajectorySequenceBuilder(new Pose2d(36, -65.5, Math.toRadians(270))).
                setReversed(true).
                forward(-25.5).
                strafeLeft(-26.5).
                build();

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        secventa2 = drive.trajectorySequenceBuilder(new Pose2d(35, -63.5, Math.toRadians(270))).
                setReversed(true).
                forward(-30).
                build();
        secventa_con1 = drive.trajectorySequenceBuilder(new Pose2d(36, -65.5, Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .back(-19)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(45)))
//                .addSpatialMarker(new Vector2d(48, -12), () -> {
//                    robot.lift.target = robot.RIDICARE_POS_2;
//                    robot.setVFBPosition(robot.VFB_ALIGN_POSE);
//                    //cod gheara
//                    robot.lift.update();
//                })
//                .addSpatialMarker(new Vector2d(45, -12), () -> {
//                    robot.lift.target = robot.RIDICARE_POS_3;
//                    robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
//                    //codgheara
//                    robot.lift.update();
//                })
                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(180)))
                .build();

        secventa_con2 = drive.trajectorySequenceBuilder(new Pose2d(36, -65.5, Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .back(-19)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(45)))
//                .addSpatialMarker(new Vector2d(48, -12), () -> {
//                    robot.lift.target = robot.RIDICARE_POS_2;
//                    robot.setVFBPosition(robot.VFB_ALIGN_POSE);
//                    //cod gheara
//                    robot.lift.update();
//                })
//                .addSpatialMarker(new Vector2d(45, -12), () -> {
//                    robot.lift.target = robot.RIDICARE_POS_3;
//                    robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
//                    //codgheara
//                    robot.lift.update();
//                })

                .build();

        secventa_con2 = drive.trajectorySequenceBuilder(new Pose2d(36, -65.5, Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .back(-19)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(45)))
                /*.addSpatialMarker(new Vector2d(48, -12), () -> {
                    robot.lift.target = robot.RIDICARE_POS_2;
                    robot.setVFBPosition(robot.VFB_ALIGN_POSE);
                    //cod gheara
                    robot.lift.update();
                })
                .addSpatialMarker(new Vector2d(45, -12), () -> {
                    robot.lift.target = robot.RIDICARE_POS_3;
                    robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
                    //codgheara
                    robot.lift.update();
                })*/
                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(180)))
                .build();


        //     .splineToConstantHeading(new Vector2d(10, -35), Math.toRadians(90))
        //     .splineToConstantHeading(new Vector2d(36,-35),Math.toRadians(270))
        //    .splineToConstantHeading(new Vector2d(43,-30), Math.toRadians(40))
        //    .splineToConstantHeading(new Vector2d(64,-31),Math.toRadians(270))
        //  .splineToConstantHeading(new Vector2d(-63,31, Math.toRadians(270)))


    }


    @Override
    public void start() {
        ArrayList<AprilTagDetection> currentDetections = aprilRecognition.getLatestDetections();
        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == CASE_1) drive.followTrajectorySequence(secventa_con1);
                else if (tag.id == CASE_3) drive.followTrajectorySequence(secventa_con3);
            }
        } else
            drive.followTrajectorySequence(secventa_con2);

        robot.start();


    }

    public void loop() {
        drive.update();
        start();
    };
}
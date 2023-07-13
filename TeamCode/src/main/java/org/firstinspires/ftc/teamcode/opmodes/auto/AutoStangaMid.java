package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.camera.pipelines.AprilRecognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "A", preselectTeleOp = "TeleOP")
public class AutoStangaMid extends LinearOpMode {

    final static double TAGSIZE = 0.166;
    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;
    private static int parkcase = 2;
    public Ridicare lift;
    AutoDreapta.State currentState = AutoDreapta.State.START;
    RobotHardware robot = new RobotHardware(this);
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    AprilRecognition aprilRecognition;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int cameraMonitorViewid;
    Pose2d startPos;
    ElapsedTime liftTimer;
    TrajectorySequence preload;
    TrajectorySequence stack_align;
    TrajectorySequence stack_forward_slow;
    TrajectorySequence stack_forword_fast;
    TrajectorySequence park_align;
    TrajectorySequence PARK_CASE1;
    TrajectorySequence PARK_CASE2;
    TrajectorySequence TRAJ_DEFAULT;
    TrajectorySequence intake;
    TrajectorySequence score;

    private void initCamera() {
        cameraMonitorViewid = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam
                (hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewid);
        aprilRecognition = new AprilRecognition(TAGSIZE, fx, fy, cx, cy);

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

    }

    private void intake() {
        robot.lift.reference = 600;
        robot.virtualFourBar.setPosition(robot.VFB_STACK_POSE);
        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
    }

    private void outtake() {
        robot.lift.reference = Ridicare.POS_2;
        robot.virtualFourBar.setPosition(robot.VFB_OUTTAKE_POSE);
        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_HIGH);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        initCamera();
        robot.claw.setPosition(0.6);
        robot.claw_alligner.setPosition(0.48);
        robot.virtualFourBar.setPosition(0.13);
        int conesStacked = 0;

        drive = new SampleMecanumDrive(hardwareMap);
        startPos = new Pose2d(-35.5, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPos);
        robot.lift.resetEncoders();

        preload = drive.trajectorySequenceBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-37, -28, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .addDisplacementMarker(() -> {robot.lift.reference = Ridicare.POS_3;})
                .splineToSplineHeading(new Pose2d(-31, -8, Math.toRadians(230)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        intake = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(4, () -> {robot.lift.reference = 600;})
                .setTangent(Math.toRadians(210))
                .splineToSplineHeading(new Pose2d(-58, -11.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        score = drive.trajectorySequenceBuilder(intake.end())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-31, -8, Math.toRadians(230)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();


        /*park_align = drive.trajectoryBuilder(stack_forward_slow.end())
                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(
                                40,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();*/


        PARK_CASE1 = drive.trajectorySequenceBuilder(score.end())
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-58, -11.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();
        PARK_CASE2 = drive.trajectorySequenceBuilder(score.end())
                .lineToLinearHeading(new Pose2d(-38, -13, Math.toRadians(183)))
                .build();
        TRAJ_DEFAULT = drive.trajectorySequenceBuilder(score.end())
                .lineToLinearHeading(new Pose2d(-35.5, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .build();

        ElapsedTime timer = new ElapsedTime();

        if (isStopRequested()) return;
        waitForStart();

        ArrayList<AprilTagDetection> currentDetections = aprilRecognition.getLatestDetections();
        if (currentDetections.size() != 0) {

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == CASE_1) parkcase = 1;
                else if (tag.id == CASE_2) {
                    parkcase = 2;
                } else if (tag.id == CASE_3) parkcase = 3;
            }
        }


        currentState = AutoDreapta.State.START;
        new Thread(() -> camera.stopStreaming()).start();
        drive.followTrajectorySequenceAsync(preload);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {
                        currentState = AutoDreapta.State.REACHED_JUNCTION;
                        robot.lift.reference = Ridicare.POS_3;
                        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_HIGH);
                        robot.virtualFourBar.setPosition(robot.VFB_HIGH);
                    }
                    break;

                case REACHED_JUNCTION:
                    if (Math.abs(-robot.lift.getCurrentPosition() - robot.lift.reference) < 800) {
                        timer.reset();
                        currentState = AutoDreapta.State.RELEASE_PRELOAD;
                    }
                    break;

                case RELEASE_PRELOAD:
                    if (timer.seconds() > 2 && timer.seconds() < 4) {
                        robot.claw.setPosition(robot.GHEARA_DESCHISA);
                    }
                    if (timer.seconds() >= 4) {
                        robot.claw.setPosition(robot.GHEARA_INCHISA);
                        robot.virtualFourBar.setPosition(0.4);
                        drive.followTrajectorySequenceAsync(intake);
                        currentState = AutoDreapta.State.INTAKE_STACK;
                    }
                    break;

                case INTAKE_STACK:
                    intake();
                    if (Math.abs(-robot.lift.getCurrentPosition() - robot.lift.reference) < 600) {
                        robot.claw.setPosition(robot.GHEARA_DESCHISA);
                        drive.followTrajectorySequenceAsync(score);
                    }
                    if (!drive.isBusy()) {
                        currentState = AutoDreapta.State.SCORE;
                    }
                    break;

                case SCORE:
                    if(conesStacked < 2) {
                        outtake();
                        drive.followTrajectorySequenceAsync(score);
                        if (drive.isBusy()) {
                            timer.reset();
                        } else if (timer.milliseconds() > 250 && timer.milliseconds() < 400) {
                            robot.claw.setPosition(robot.GHEARA_DESCHISA);
                        } else {
                            robot.claw.setPosition(robot.GHEARA_INCHISA);
                            currentState = AutoDreapta.State.INTAKE_STACK;
                            conesStacked++;
                        }
                    } else currentState = AutoDreapta.State.PARK;
                    break;


                case PARK:
                    if (parkcase == 1) {
                        drive.followTrajectorySequenceAsync(PARK_CASE1);
                    } else if (parkcase == 2) {
                        drive.followTrajectorySequenceAsync(PARK_CASE2);
                    } else {
                        drive.followTrajectorySequenceAsync(TRAJ_DEFAULT);
                    }


            }
            drive.update();
            telemetry.addData("state", currentState);
            telemetry.addData("Ridicare", robot.lift.getCurrentPosition());
            telemetry.update();
            robot.lift.update();

        }
    }
}






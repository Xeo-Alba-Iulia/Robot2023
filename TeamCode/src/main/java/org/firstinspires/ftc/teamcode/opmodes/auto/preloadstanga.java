package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.camera.pipelines.AprilRecognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "A", preselectTeleOp = "TeleOP")
public class preloadstanga extends LinearOpMode {

    final static double TAGSIZE = 0.166;
    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;
    private static int parkcase = 2;



    public Ridicare lift;
    AutoDreapta.State currentState = AutoDreapta.State.IDLE;
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
    Trajectory preload;
    Trajectory stack_align;
    Trajectory stack_forward_slow;
    Trajectory stack_forword_fast;
    Trajectory park_align;
    Trajectory PARK_CASE1;
    Trajectory PARK_CASE2;
    Trajectory TRAJ_DEFAULT;


    private void initCamera() {
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

    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        initCamera();
        robot.claw.setPosition(0.6);
        robot.claw_alligner.setPosition(0.48);
        robot.virtualFourBar.setPosition(0.13);

        drive = new SampleMecanumDrive(hardwareMap);
        startPos = new Pose2d(-35.5, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPos);

        preload = drive.trajectoryBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-35, -30, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .splineToSplineHeading(new Pose2d(-28, -5, Math.toRadians(225)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        stack_align = drive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(-46, -11, Math.toRadians(0)))
                .build();

        stack_forword_fast = drive.trajectoryBuilder(stack_align.end())
                .lineToLinearHeading(new Pose2d(-49.8, -11, Math.toRadians(0)))
                .build();

        stack_forward_slow = drive.trajectoryBuilder(stack_forword_fast.end())
                .lineToLinearHeading(new Pose2d(-52.8, -11, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(
                                10,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        park_align = drive.trajectoryBuilder(stack_forward_slow.end())
                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(
                                40,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        PARK_CASE1 = drive.trajectoryBuilder(park_align.end())
                .back(24)
                .build();
        PARK_CASE2 = drive.trajectoryBuilder(park_align.end())
                .strafeLeft(10)
                .build();
        TRAJ_DEFAULT = drive.trajectoryBuilder(park_align.end())
                .forward(24)
                .build();

        double waitTime2 = 3;
        double waitTime1 = 0.75;
        ElapsedTime timer = new ElapsedTime();


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

        drive.followTrajectoryAsync(preload);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {
                        currentState = AutoDreapta.State.REACHED_JUNCTION;
                        robot.lift.target = Ridicare.POS_3;
                        robot.virtualFourBar.setPosition(robot.VFB_MEDIUM);
                    }
                    break;

                case REACHED_JUNCTION:

                    if (Math.abs(-robot.lift.getCurrentPosition() - robot.lift.target) < 800) {
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
                        robot.lift.target = 600;
                        robot.virtualFourBar.setPosition(0.4);

//                        drive.followTrajectory(stack_align);
                        drive.followTrajectoryAsync(park_align);
                        currentState = AutoDreapta.State.PARK;
                    }
                    break;

                case PARK:
                    if (parkcase == 1) {
                        drive.followTrajectoryAsync(PARK_CASE1);
                    } else if (parkcase == 2) {
                        drive.followTrajectoryAsync(PARK_CASE2);
                    } else {
                        drive.followTrajectoryAsync(TRAJ_DEFAULT);
                    }


            }
            drive.update();
            telemetry.addData("state", currentState);
            telemetry.addData("Ridicare", robot.lift.getCurrentPosition());
            telemetry.update();
            robot.lift.update();
            if (isStopRequested()) {
                return;
            }

        }
    }
}






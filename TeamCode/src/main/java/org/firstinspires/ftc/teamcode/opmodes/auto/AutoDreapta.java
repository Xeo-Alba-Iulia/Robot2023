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

@Autonomous(group = "C", preselectTeleOp = "TeleOp")
public class StateAutonomous extends LinearOpMode {
    final static double TAGSIZE = 0.166;
    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;
    private static int parkcase = 2;
    public Ridicare lift;
    State currentState = State.IDLE;
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
    public void runOpMode() {

        initCamera();
        robot.init();
        robot.claw.setPosition(0.6);
        robot.claw_alligner.setPosition(0.48);
        robot.virtualFourBar.setPosition(0.13);

        drive = new SampleMecanumDrive(hardwareMap);
        startPos = new Pose2d(38, -65.5, Math.toRadians(270));
        drive.setPoseEstimate(startPos);


        Trajectory preload = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(27.6, -32.4, Math.toRadians(315)),
                        SampleMecanumDrive.getVelocityConstraint(
                                45,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory park_align = drive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(0)))
                .build();


        Trajectory stack = drive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(0)))
                .build();

        Trajectory PARK_CASE1 = drive.trajectoryBuilder(park_align.end())
                .forward(10)
                .build();
        Trajectory PARK_CASE2 = drive.trajectoryBuilder(park_align.end())
                .back(10)
                .build();
        Trajectory TRAJ_DEFAULT = drive.trajectoryBuilder(park_align.end())
                .forward(10)
                .build();


//        Trajectory CORRECT_CASE;
//        Trajectory mediumJunction = drive.trajectoryBuilder(stack.end())
//                .lineTo(new Vector2d(38, -13))
//                .lineToLinearHeading(new Pose2d(33, -15, Math.toRadians(25)))
//                .build();

//
//        Trajectory allignStack = drive.trajectoryBuilder(mediumJunction.end())
//                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(0)))
//                .lineTo(new Vector2d(57, -12))
//                .build();
//
//        Trajectory Case3 = drive.trajectoryBuilder(mediumJunction.end())
//                .lineToLinearHeading(new Pose2d(60,-15, Math.toRadians(0)))
//                .build();
//
//        Trajectory Case1 = drive.trajectoryBuilder(mediumJunction.end())
//                .lineToLinearHeading(new Pose2d(10, -15, Math.toRadians(0)))
//                .build();
        double waitTime2 = 3;
        double waitTime1 = 0.75;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime waitTimer2 = new ElapsedTime();

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

        if (isStopRequested()) return;
        currentState = State.START;
        drive.followTrajectory(preload);


        while (opModeIsActive() && !isStopRequested()) {


            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case START:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    robot.lift.target = Ridicare.POS_2;
                    robot.virtualFourBar.setPosition(robot.VFB_MEDIUM);


                    if (!drive.isBusy()) {
                        currentState = State.REACHED_JUNCTION;
                        timer.reset();

                    }
                    break;

                case REACHED_JUNCTION:

                    if (robot.lift.getCurrentPosition() - robot.lift.target < 800) {
                        timer.reset();
                    }

                    if (timer.seconds() > 1.5) {
                        robot.claw.setPosition(robot.GHEARA_DESCHISA);
                        timer.reset();
                    }


                    if (timer.seconds() > 0.4) {
                        robot.claw.setPosition(robot.GHEARA_INCHISA);
                        robot.lift.target = 600;
                        robot.virtualFourBar.setPosition(robot.VFB_STACK_POSE - 0.04);
                        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
                        currentState = State.STACK;
                        drive.followTrajectory(stack);
                    }

                    break;

                case STACK:

                    robot.claw.setPosition(robot.GHEARA_DESCHISA);
                    if(!drive.isBusy()) {
                        currentState = State.IDLE;
                    }

                    break;


                case IDLE:

                    break;

            }
            drive.update();

            telemetry.addData("state", currentState);
            robot.lift.update();
        }

    }

    enum State {
        START,
        REACHED_JUNCTION,
        STACK,
        WAIT_3,
        MEDIUMJUNCTION,
        WAIT_2,
        ALIGNSTATE,
        WAIT_4,
        PARK,
        PARK_ALIGN,
        IDLE

    }


}


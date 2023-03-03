package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.camera.pipelines.AprilRecognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "C", preselectTeleOp = "TeleOp")
public class StateAutonomous extends LinearOpMode {
    final static double TAGSIZE = 0.166;
    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;
    enum State {
        PRELOAD,
        WAIT_1,
        STACK,
        WAIT_3,
        MEDIUMJUNCTION,
        WAIT_2,
        ALIGNSTATE,
        WAIT_4,
        IDLE
    }

    State currentState = State.IDLE;
    Trajectory preload;

    Trajectory stack;

    RobotHardware robot;

    Trajectory mediumJunction;

    Trajectory allignStack;

    OpenCvCamera camera;
    AprilRecognition aprilRecognition;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int cameraMonitorViewid;
    Pose2d startPos= new Pose2d(32, -65.5, Math.toRadians(270));

    Trajectory preload2;


    @Override
    public void runOpMode() {
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
        SampleMecanumDrive drive;
        drive = new SampleMecanumDrive(hard


        preload = drive.trajectoryBuilder(startPos)
                //.lineToLinearHeading(new Pose2d(35, -21, Math.toRadians(0)))
                .back(10)
                .build();
        preload2 = drive.trajectoryBuilder(preload.end())
                .forward(10)
                .build();




        stack = drive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(35,-12, Math.toRadians(0)))
                .build();

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
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;
        currentState = State.PRELOAD;
        drive.followTrajectory(preload);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case PRELOAD:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        waitTimer1.reset();



                    }
                    break;
                case WAIT_1:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState =  State.STACK;
//                        robot.lift.target= Ridicare.POS_2;
//                        robot.virtualFourBar.setPosition(robot.VFB_MEDIUM);
//                        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
//                        drive.followTrajectoryAsync(stack);


                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case STACK:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        waitTimer1.reset();
                    }
                    break;
//
//
//


//                case WAIT_2:
//
//                    if (waitTimer1.seconds() >= waitTime1) {
//                        currentState = State.MEDIUMJUNCTION;
//                        ///hai deluta, hai suuuus ,deluta de mariii
//                    }
//                    break;
//
//
//                case MEDIUMJUNCTION:
//                    if (!drive.isBusy()) {
//                        currentState = State.WAIT_3;
//                        waitTimer1.reset();
//                    }
//                    break;
//                case WAIT_3:
//
//
//                    if (waitTimer1.seconds() >= waitTime1) {
//                        currentState = State.ALIGNSTATE;
//                        ///hai deluta, hai suuuus ,deluta de mariii
//                    }
//                    break;
//                case ALIGNSTATE:
//                    if (!drive.isBusy()) {
//                        currentState = State.WAIT_4;
//                        waitTimer1.reset();
//                    }
//                    break;
//                case WAIT_4:
//                    if (waitTimer1.seconds() >= waitTime1) {
//                        currentState = State.IDLE;
//                        ///hai deluta, hai suuuus ,deluta de mariii
//                    }
//                    break;
//                case IDLE:
//
//                    break

            }
            drive.update();

            robot.lift.update();
        }

    }
}


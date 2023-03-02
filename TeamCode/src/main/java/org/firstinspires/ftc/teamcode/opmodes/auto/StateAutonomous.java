package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;

@Autonomous(group = "C", preselectTeleOp = "TeleOp")
public class StateAutonomous extends LinearOpMode {
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


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive;
        drive = new SampleMecanumDrive(hardwareMap);



        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();

        preload = drive.trajectoryBuilder(new Pose2d(32, -65.5, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(35, -21, Math.toRadians(0)))
                .build();

        stack = drive.trajectoryBuilder(new Pose2d(35, -21, Math.toRadians(0)))
                .strafeRight(7)
                .lineTo(new Vector2d(57, -12))
                .build();

        mediumJunction = drive.trajectoryBuilder(new Pose2d(57, -12, Math.toRadians(0)))
                .lineTo(new Vector2d(38, -13))
                .lineToLinearHeading(new Pose2d(33, -15, Math.toRadians(25)))
                .build();

        allignStack = drive.trajectoryBuilder(new Pose2d(33, -15, Math.toRadians(25)))
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(0)))
                .lineTo(new Vector2d(57, -12))
                .build();

        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;
        currentState = State.PRELOAD;
        drive.followTrajectoryAsync(preload);

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
                        robot.lift.target= Ridicare.POS_2;
                        robot.virtualFourBar.setPosition(robot.VFB_MEDIUM);
                        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
                        drive.followTrajectoryAsync(stack);


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
                case WAIT_2:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.MEDIUMJUNCTION;
                        ///hai deluta, hai suuuus ,deluta de mariii
                    }
                    break;


                case MEDIUMJUNCTION:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_3;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.ALIGNSTATE;
                        ///hai deluta, hai suuuus ,deluta de mariii
                    }
                    break;
                case ALIGNSTATE:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_4;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_4:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.IDLE;
                        ///hai deluta, hai suuuus ,deluta de mariii
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
        }
        drive.update();
        // We update our lift PID continuously in the background, regardless of state
        robot.lift.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Continually write pose to `PoseStorage`

        PoseStorage.currentPose = poseEstimate;


        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}


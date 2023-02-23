package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.posStorage;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    boolean stackToggle;

    RobotHardware robot = new RobotHardware(this);
    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    TrajectorySequence junction1;

    @Override
    public void init() {
        stackToggle = false;
        robot.init();
        robot.claw.setPosition(robot.GHEARA_DESCHISA);
        robot.vFB1.setPosition(0);
        robot.vFB2.setPosition(1);
        myLocalizer.setPoseEstimate(posStorage.currentPose);
    }

    @Override
    public void start() {
        robot.start();
    }

//    public void teleOpRoadRunner() {
//
//
//        // Set your initial pose to x: 10, y: 10, facing 90 degrees
//        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
//
////        waitForStart();
//
//
//            // Make sure to call drive.update() on *every* loop
//            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
//
//
//            // Retrieve your pose
//
//
//            telemetry.addData("x", myPose.getX());
//            telemetry.addData("y", myPose.getY());
//            telemetry.addData("heading", myPose.getHeading());
//
//            // Insert whatever teleop code you're using
//
//    }

    public void allignJunction() {
        myLocalizer.setPoseEstimate(posStorage.currentPose);
        drive.setPoseEstimate(posStorage.currentPose);
        Pose2d myPose = drive.getPoseEstimate();
        //sau
//         Pose2d myPose = myLocalizer.getPoseEstimate();
        Pose2d posjunction1 = new Pose2d(-31, 8, 135);

        junction1 = drive.trajectorySequenceBuilder(myPose)
                .lineToLinearHeading(posjunction1)
                .build();

    }

    private void intakeOuttake() {
        if (gamepad2.dpad_down) {
            robot.lift.target = 600;
            robot.setVFBPosition(robot.VFB_ALIGN_POSE);
        } else if (gamepad2.dpad_left) {
            robot.lift.target = robot.RIDICARE_POS_1;
            robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.dpad_up) {
            robot.lift.target = robot.RIDICARE_POS_2;
            robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.dpad_right) {
            robot.lift.target = robot.RIDICARE_POS_3;
            robot.setVFBPosition(robot.VFB_ALIGN_HIGH_POSE);
        }

        robot.lift.update();


        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            robot.lift.update();
        } else {
            robot.lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            robot.lift.target = robot.lift.getCurrentPosition();
        }
    }

    private void virtualFourBar() {
        if (gamepad2.left_bumper) {
            robot.setVFBPosition(robot.VFB_INTAKE_POSE);
        } else if (gamepad2.right_stick_button) {
            robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.right_bumper) {
            robot.setVFBPosition(robot.VFB_ALIGN_POSE);
        }
    }

    private void claw() {
        if (gamepad2.a) {
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if (gamepad2.b) {
            robot.claw.setPosition(robot.GHEARA_DESCHISA);
        }
    }

    private void stack() {

        if (gamepad1.a && !stackToggle) {
            stackToggle = true;
            robot.vfb_stack_pose -= 0.07;
            robot.setVFBPosition(robot.vfb_stack_pose);
        } else if (gamepad1.y) {
            stackToggle = true;;
            robot.vfb_stack_pose += 0.07;
            robot.setVFBPosition(robot.vfb_stack_pose);
        } else if ((!gamepad1.a || gamepad1.b) && stackToggle) {
            stackToggle = false;
        }
    }

    private void telemetry() {
        telemetry.addLine("Ridicare");
        telemetry.addData("Pozitie", robot.lift.getCurrentPosition());
        telemetry.addLine("Poz Vfb");
        telemetry.addData("Stack", robot.vfb_stack_pose);
        telemetry.update();
    }


    public void loop() {
        myLocalizer.update();
        drive.update();
        robot.movement(gamepad1);
        intakeOuttake();
        virtualFourBar();
        claw();
        allignJunction();
//        teleOpRoadRunner();
//        stack();

        telemetry();
        telemetry.update();
    }

}

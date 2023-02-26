package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.posStorage;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    boolean stackToggle;
    boolean clawToggle;
    boolean currentA;

    RobotHardware robot = new RobotHardware(this);
    StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;
    TrajectorySequence junction1;

    @Override
    public void init() {
        stackToggle = false;
        clawToggle = false;
        currentA = false;
        robot.init();
        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_UP);
        robot.claw.setPosition(robot.GHEARA_INIT);
        robot.vFB1.setPosition(0);
        robot.vFB2.setPosition(1);
        drive = new SampleMecanumDrive(hardwareMap);
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        myLocalizer.setPoseEstimate(posStorage.currentPose);
    }


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

        boolean manualActive = gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0;


        if (!manualActive) {
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

    private void allignclaw() {
        if (gamepad2.x) {
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_UP);
        } else if (gamepad2.y) {
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
        } else if (gamepad2.right_bumper)
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_FALLEN);
    }

    private void claw() {
        if (gamepad1.a != currentA) {
            currentA = gamepad1.a;
            if (gamepad1.a) {
                clawToggle = !clawToggle;
                if (clawToggle) {
                    robot.claw.setPosition(robot.GHEARA_INCHISA);
                } else {
                    robot.claw.setPosition(robot.GHEARA_DESCHISA);
                }
            }
        }
    }


    private void stack() {

        if (gamepad1.a && !stackToggle) {
            stackToggle = true;
            robot.vfb_stack_pose -= 0.07;
            robot.setVFBPosition(robot.vfb_stack_pose);
        } else if (gamepad1.y) {
            stackToggle = true;
            robot.vfb_stack_pose += 0.07;
            robot.setVFBPosition(robot.vfb_stack_pose);
        } else if ((!gamepad1.a || gamepad1.y) && stackToggle) {
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
        allignclaw();
//        stack();

        telemetry();
        telemetry.update();
    }

}

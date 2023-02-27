package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.posStorage;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    boolean stackToggle;
    boolean intakeToggle;
    boolean clawToggle;
    boolean currentA;
    boolean currentX;
    boolean currentDdown;



    RobotHardware robot = new RobotHardware(this);
    StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;
    TrajectorySequence junction1;

    @Override
    public void init() {
        stackToggle = false;
        clawToggle = false;
        intakeToggle = false;
        currentA = false;
        currentX = false;
        currentDdown=false;
        robot.init();
        robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
        robot.claw.setPosition(robot.GHEARA_DESCHISA);
        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
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

    private void intake() {
        if (gamepad2.dpad_down != currentDdown) {
            currentDdown = gamepad1.a;
            if (gamepad2.dpad_down) {
                intakeToggle = !intakeToggle;
                if (intakeToggle) {
                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
                } else {
                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
                }
            }
        }
    }
    private void intakeOuttake() {
        if (gamepad2.dpad_down) {
            robot.lift.target = 600;
            robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if (gamepad2.dpad_left) {
            robot.lift.target = Ridicare.POS_1;
            robot.virtualFourBar.setPosition(robot.VFB_LOW);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if (gamepad2.dpad_up) {
            robot.lift.target = Ridicare.POS_2;
            robot.virtualFourBar.setPosition(robot.VFB_MEDIUM);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if (gamepad2.dpad_right) {
            robot.lift.target = Ridicare.POS_3;
            robot.virtualFourBar.setPosition(robot.VFB_HIGH);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_HIGH);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        }

            robot.lift.update();
    }

    private void virtualFourBar() {
        if (gamepad1.left_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_INTAKE_POSE);
        } else if (gamepad2.right_stick_button) {
            robot.virtualFourBar.setPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.right_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
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
    private void cazut() {
            if (gamepad1.x != currentX) {
                currentX = gamepad1.x;
                if (gamepad1.x) {
                    clawToggle = !clawToggle;
                    if (clawToggle) {
                        robot.claw.setPosition(robot.VFB_FALLEN);
                        robot.virtualFourBar.setPosition(robot.VFB_FALLEN);
                        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_FALLEN);
                        robot.lift.target = 600;
                    } else {
                        robot.claw.setPosition(robot.VFB_ALIGN_POSE);
                        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);

                    }
                }
            }
            robot.lift.update();
        }

    private void stackModifier()  {
       if(gamepad1.b) {
           robot.VFB_STACK_POSE += 0.005;
           robot.virtualFourBar.setPosition(robot.VFB_STACK_POSE);
       }
       else if(gamepad1.y) {
           robot.VFB_STACK_POSE -= 0.005;
           robot.virtualFourBar.setPosition(robot.VFB_STACK_POSE);
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
        cazut();
        stackModifier();

        telemetry();
        telemetry.update();
    }

}

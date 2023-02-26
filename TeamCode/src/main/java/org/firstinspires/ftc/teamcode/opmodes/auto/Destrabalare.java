package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class Destrabalare extends OpMode {

    RobotHardware robot;

    Pose2d startPose = new Pose2d(35, -60.5, Math.toRadians(270));
    //Create elapsed time variable and an instance of elapsed time
    SampleMecanumDrive drive;
    Pose2d firstJunction = new Pose2d(36, -31, Math.toRadians(330));
    Pose2d alignStack = new Pose2d(36, -10, 0);
    Vector2d stack = new Vector2d(57, -8);
    Pose2d midCycle = new Pose2d(33, -9, Math.toRadians(45));



    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotHardware(this);
        robot.init();
        drive.setPoseEstimate(startPose);

        robot.claw.setPosition(robot.GHEARA_INCHISA);
        Trajectory toFirstJunction = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    robot.lift.target = Ridicare.POS_2;
                })
                .lineToLinearHeading(firstJunction)
                .addDisplacementMarker(3, () -> {
                    robot.claw.setPosition(robot.GHEARA_DESCHISA);
                })
                .build();



        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.claw.setPosition(robot.GHEARA_DESCHISA);
                })
//                .lineToLinearHeading(alignStack)
//                .splineToConstantHeading(stack, Math.toRadians(30))
//                .lineToLinearHeading(midCycle)
                .build();

        drive.followTrajectoryAsync(toFirstJunction);

    }

    @Override
    public void loop() {
        drive.update();
        robot.lift.update();
    }

}

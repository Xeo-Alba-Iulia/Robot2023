package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "medii", group = "A", preselectTeleOp = "TeleOP")
public class Medii extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(35, -60.355, Math.toRadians(270));
        Trajectory inceput = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(315)))
                .build();
        Trajectory align = drive.trajectoryBuilder(inceput.end())
                .lineToLinearHeading(new Pose2d(36,-12, 0))
                .build();
        Trajectory cone = drive.trajectoryBuilder(align.end())
                .forward(18)
                .build();
        Trajectory mid = drive.trajectoryBuilder(cone.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(45)))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(inceput);
        drive.followTrajectory(align);
        drive.followTrajectory(cone);
        drive.followTrajectory(mid);


    }
}

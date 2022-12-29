package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AutoUsingDetector_Right", group="Tutorials")

public class roadrunnertest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-48, -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        // distances are in Inches
        // a blank Pose2d(0 is the same as Pose2d(0, 0, 0) x = 0, y = 0, rads = 0

        /* Will throw PathContinuityException
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();
         */

        /* Will Run Two Different trajectories - works, but not good.
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5)
                .build();
         */

        /* Makes Smooth Paths  Sometimes weird
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x1, y1), heading)
                .splineTo(new Vector2d(x2, y2), heading)
                .build();
         */

        waitForStart();

        if (isStopRequested()) return;


        //drive.followTrajectory(traj);
    }

}

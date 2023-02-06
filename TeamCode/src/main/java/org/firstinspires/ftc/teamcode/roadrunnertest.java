package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(name="roadrunnertest", group="Tutorials")

public class roadrunnertest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double startDir = Math.toRadians(90);

        Pose2d startPose = new Pose2d(-40, -65, Math.toRadians(90));

        Pose2d startPose2 = new Pose2d(-0, -28, Math.toRadians(90));
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


        /*
        Trajectory trajectory =  drive.trajectoryBuilder(new Pose2d(-36, -64, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-13, -54), startDir)
                .lineTo(new Vector2d(-13, -42))
                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))
                //.setTangent(48)
                .splineToConstantHeading(new Vector2d(-12, -32), 48)
                .splineTo(new Vector2d(-12, -12), startDir)
                //.setTangent(22)
                .splineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)), 22)
                .lineTo(new Vector2d(-60, -12))
                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                //.waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                //.waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                //.waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                //.waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                .build();
         */


        TrajectorySequence traj1  = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(10))
                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                .lineToConstantHeading( new Vector2d(-12, -40))
                .splineToConstantHeading(new Vector2d(2, -32), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(2, -30))
                .lineToConstantHeading(new Vector2d(2, -36))
                .lineToConstantHeading(new Vector2d(-12, -36))
                .lineToLinearHeading(new Pose2d(-12, -13, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-60, -13))
                .build();


        TrajectorySequence traj2  = drive.trajectorySequenceBuilder( traj1.end() )
                .lineToSplineHeading(new Pose2d(-19, -7.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-60, -13, Math.toRadians(180)))
                .build();


        waitForStart();

        if (isStopRequested()) return;


      //  for (double i = 0; i < 2; i++){

        drive.followTrajectorySequence(traj1);
        sleep(500);
        for (int i = 0; i < 5; i++){
            drive.followTrajectorySequence(traj2);
        }
        //drive.followTrajectorySequence(traj2);
          //  drive.followTrajectorySequence(traj2);
          //  sleep(500);
      //  }
    }

}

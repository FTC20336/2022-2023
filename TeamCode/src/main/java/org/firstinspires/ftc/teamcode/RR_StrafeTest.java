package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="ARR Strafe", group="Right")

public class RR_StrafeTest extends LinearOpMode {

    public static double  strafeDistance = 36;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive  = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1  = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(strafeDistance)
                .build();

        TrajectorySequence traj2  = drive.trajectorySequenceBuilder(traj1.end())
                .strafeRight(strafeDistance)
                .build();

        waitForStart();
        telemetry.addData("Strafing About to Strafe left, then right after 5 second..\n Measure strafe distance to Calculate Lateral Multiplier", strafeDistance);
        telemetry.update();

        drive.followTrajectorySequence(traj1);
        sleep(1000);
        drive.followTrajectorySequence(traj2);

    }

}
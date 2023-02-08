package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

@Autonomous(name="AutoUsingDetector_Left_RR", group="Tutorials")

public class AutoUsingDetector_Left_RR extends LinearOpMode {
    private OpenCvCamera webcam;

    //Create New Robot based on RobotBase
    //RobotBase Beep = new RobotBase();

    // Dont really need to change this.
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Change these values to move the little square/region where we check what color we see
    private static double RegionCenterX = 875; // Distance in pixels from the Left
    private static double RegionCenterY = 340; // Distance in pixels from the top
    private static double RegionWidth = 50;
    private static double RegionHeight = 50;

    private BBBDetector_Color.ElementPosition ParkingPos;

    @Override
    public void runOpMode() {
        //Beep.init(hardwareMap, this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double startDir = Math.toRadians(90);

        RobotArm BeepArm= new RobotArm();

        BeepArm.init(hardwareMap, this);


        Pose2d startPose = new Pose2d(-40, -65, Math.toRadians(90));

        Pose2d startPose2 = new Pose2d(-0, -28, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Color myPipeline;
        webcam.setPipeline(myPipeline = new BBBDetector_Color(RegionCenterX, RegionCenterY, RegionWidth, RegionHeight));

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera Opening Error !!!!!");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addData("Push Camera Stream and tap screen to update image \nAlign the square to the cone \n \n Wait at least 5 Sec before Pressing Start", "");
        telemetry.update();

        waitForStart();

        // Get the latest frame analyzed result
        ParkingPos = myPipeline.getAnalysis();

        TrajectorySequence traj1  = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(10))
                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                .lineToLinearHeading( new Pose2d(-10, -23, Math.toRadians(0)))
                .build();

        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-5, -24))
                .build();

        TrajectorySequence traj175 = drive.trajectorySequenceBuilder(traj15.end())
                //.splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))
                //.lineToConstantHeading(new Vector2d(0, -30))
                //.lineToConstantHeading(new Vector2d(0, -36))
                //.lineToConstantHeading(new Vector2d(-12, -36))
                .lineToConstantHeading(new Vector2d(-12, -24))
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                //.lineToConstantHeading(new Vector2d(-60, -12))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(traj175.end())
                .lineToConstantHeading(new Vector2d(-36, -12))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(traj175.end())
                .lineToConstantHeading(new Vector2d(-60, -12))
                .build();



        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            telemetry.addLine("Autonomous A - Blue - Right");
            telemetry.update();

            BeepArm.ClawFullClose(-1);
            BeepArm.ViperSlideSetPos(2, 2, -1);
            drive.followTrajectorySequence(traj1);
            //sleep(500);
            BeepArm.ViperSlideSetPos(32, 24, -1);
            drive.followTrajectorySequence(traj15);
            BeepArm.ClawFullOpen(-1);
            BeepArm.ViperSlideSetPos(0, 24, -1);
            drive.followTrajectorySequence(traj175);

            //AUTONOMOUS_A();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            telemetry.addLine("Autonomous B - Green - Center");
            telemetry.update();

            BeepArm.ClawFullClose(-1);
            BeepArm.ViperSlideSetPos(2, 2, -1);
            drive.followTrajectorySequence(traj1);
            //sleep(500);
            BeepArm.ViperSlideSetPos(32, 24, -1);
            drive.followTrajectorySequence(traj15);
            BeepArm.ClawFullOpen(-1);
            BeepArm.ViperSlideSetPos(0, 24, -1);
            drive.followTrajectorySequence(traj175);

            drive.followTrajectorySequence(center);
            //AUTONOMOUS_B();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            telemetry.addLine("Autonomous C - Yellow - Left");
            telemetry.update();

            BeepArm.ClawFullClose(-1);
            BeepArm.ViperSlideSetPos(2, 2, -1);
            drive.followTrajectorySequence(traj1);
            //sleep(500);
            BeepArm.ViperSlideSetPos(32, 24, -1);
            drive.followTrajectorySequence(traj15);
            BeepArm.ClawFullOpen(-1);
            BeepArm.ViperSlideSetPos(0, 24, -1);
            drive.followTrajectorySequence(traj175);

            drive.followTrajectorySequence(left);
            //AUTONOMOUS_C();
        }
    }
}
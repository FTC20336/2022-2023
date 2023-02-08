package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
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
@Autonomous(name="AutoUsingDetector_Right_RR", group="Right")

public class AutoUsingDetector_Right_RR extends LinearOpMode {
    private OpenCvCamera webcam;


    RobotArm BeepArm= new RobotArm();

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
    public static double startx = 34.5;
    public static double stackh = 5;
    public static double stackinc = 1.25;

    double startDir = Math.toRadians(90);


    Pose2d startPose = new Pose2d(startx, -65, Math.toRadians(90));


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive  = new SampleMecanumDrive(hardwareMap);


        TrajectorySequence traj1  = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(170))
                .splineToConstantHeading(new Vector2d(12, -52), Math.toRadians(90))
                .lineToLinearHeading( new Pose2d(12, -24, Math.toRadians(180)))
                .build();

        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(8, -24),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence traj175 = drive.trajectorySequenceBuilder(traj15.end())
                //.back(4)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24, -12), Math.toRadians(0))
                //.lineToConstantHeading(new Vector2d(15, -24))
                //.lineToLinearHeading(new Pose2d(15, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence setupCycle = drive.trajectorySequenceBuilder(traj175.end())
                .lineToConstantHeading(new Vector2d(61.5, -12))
                .build();

        TrajectorySequence toCyclePole = drive.trajectorySequenceBuilder(setupCycle.end())
                .lineToLinearHeading(new Pose2d(46, -10, Math.toRadians(270)))
                .build();

        TrajectorySequence front = drive.trajectorySequenceBuilder(toCyclePole.end())
                .lineToConstantHeading(new Vector2d(46, -17))
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(front.end())
                .lineToConstantHeading(new Vector2d(46, -10))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(back.end())
                .lineToLinearHeading(new Pose2d(61.5, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(36, -12))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(8, -12))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(56, -12))
                .build();

        BeepArm.init(hardwareMap, this);

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

        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            telemetry.addLine("Autonomous A - Blue - Right");
            telemetry.update();

            BeepArm.ClawFullClose(450); //wait 250 ms to make sure the cone is gripped well
            BeepArm.ViperSlideSetPos(2, 24, -1);
            BeepArm.ViperSlideSetPos(32.5, 11, 1);
            drive.followTrajectorySequence(traj1);

            BeepArm.ViperSlideSetPos(32.5, 24, 1);
            drive.followTrajectorySequence(traj15);
            sleep(250); // wait a little if the robot wiggle

            BeepArm.ViperSlideSetPos(30, 6, 1000);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(traj175);
            BeepArm.ViperSlideSetPos(stackh, 36, 1); //Don't wait.. go back now

            drive.followTrajectorySequence(setupCycle);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, -1);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
            drive.followTrajectorySequence(toStack);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, -1);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(0,24,1);

            drive.followTrajectorySequence(right);

        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            telemetry.addLine("Autonomous B - Green - Center");
            telemetry.update();

            BeepArm.ClawFullClose(450); //wait 250 ms to make sure the cone is gripped well
            BeepArm.ViperSlideSetPos(2, 24, -1);
            BeepArm.ViperSlideSetPos(32.5, 11, 1);
            drive.followTrajectorySequence(traj1);

            BeepArm.ViperSlideSetPos(32.5, 24, 1);
            drive.followTrajectorySequence(traj15);
            sleep(250); // wait a little if the robot wiggle

            BeepArm.ViperSlideSetPos(30, 6, 1000);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(traj175);
            BeepArm.ViperSlideSetPos(stackh, 36, 1); //Don't wait.. go back now

            drive.followTrajectorySequence(setupCycle);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, -1);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
            drive.followTrajectorySequence(toStack);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, -1);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(0,24,1);

            drive.followTrajectorySequence(center);

        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            telemetry.addLine("Autonomous C - Yellow - Left");
            telemetry.update();

            BeepArm.ClawFullClose(450); //wait 250 ms to make sure the cone is gripped well
            BeepArm.ViperSlideSetPos(2, 24, -1);
            BeepArm.ViperSlideSetPos(32.5, 11, 1);
            drive.followTrajectorySequence(traj1);

            BeepArm.ViperSlideSetPos(32.5, 24, 1);
            drive.followTrajectorySequence(traj15);
            sleep(250); // wait a little if the robot wiggle

            BeepArm.ViperSlideSetPos(30, 6, 1000);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(traj175);
            BeepArm.ViperSlideSetPos(stackh, 36, 1); //Don't wait.. go back now

            drive.followTrajectorySequence(setupCycle);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, -1);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
            drive.followTrajectorySequence(toStack);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, -1);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(0,24,1);

            drive.followTrajectorySequence(left);

        }

    }

}
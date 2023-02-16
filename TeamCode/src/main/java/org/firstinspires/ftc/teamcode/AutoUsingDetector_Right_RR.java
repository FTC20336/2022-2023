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
@Autonomous(name="Right_RR_TP", group="Right")

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

    public static Vector2d coneStack = new Vector2d(62.25, -17.5);
    public static Vector2d shortPole = new Vector2d(48,-21);

    public static Vector2d p1 = new Vector2d(32,-65); // Starting Point
    public static Vector2d p2 = new Vector2d(13.15,-63); //
    public static Vector2d p3 = new Vector2d(13.25,-25.5); // Right of the Tall Middle Junction
    public static Vector2d p4 = new Vector2d(9, -24.75); //Right above Pole
    public static Vector2d p5 = new Vector2d(11,-23.75); //
    public static Vector2d p6 = new Vector2d(16,  coneStack.getY() ); //
    public static long stackDelay = 500;

    public static double stackh = 5;
    public static double stackinc = 1.25;


    double startDir = Math.toRadians(90);


    Pose2d startPose = new Pose2d(p1.getX(), p1.getY(), Math.toRadians(90));


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive  = new SampleMecanumDrive(hardwareMap);


        TrajectorySequence traj1  = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(p2.getX(),p2.getY()))
                .lineToLinearHeading( new Pose2d(p3.getX(), p3.getY(), Math.toRadians(180)))
                .build();

        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(p4.getX(), p4.getY()),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence traj175 = drive.trajectorySequenceBuilder(traj15.end())
                .lineToConstantHeading(new Vector2d(p5.getX(), p5.getY()))
                .lineToLinearHeading(new Pose2d(p6.getX(), coneStack.getY(), Math.toRadians(0)))
                .build();

        TrajectorySequence setupCycle = drive.trajectorySequenceBuilder(traj175.end())
                .lineToConstantHeading(new Vector2d(coneStack.getX(), coneStack.getY()))
                .build();

        TrajectorySequence toCyclePole = drive.trajectorySequenceBuilder(setupCycle.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(shortPole.getX(), coneStack.getY(), Math.toRadians(270)))
                .build();

        TrajectorySequence front = drive.trajectorySequenceBuilder(toCyclePole.end())
                .lineToConstantHeading(new Vector2d(shortPole.getX(), shortPole.getY()))
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(front.end())
                .lineToConstantHeading(new Vector2d(shortPole.getX(), coneStack.getY()))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(back.end())
                .lineToLinearHeading(new Pose2d(coneStack.getX()-4, coneStack.getY(), Math.toRadians(0)))
                .forward(4)
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(36, coneStack.getY()))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(14, coneStack.getY()))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(58, coneStack.getY()))
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
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN); // Was Upright

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
            BeepArm.ViperSlideSetPos(16, 36, stackDelay);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
            drive.followTrajectorySequence(toStack);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, stackDelay);
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
            BeepArm.ViperSlideSetPos(16, 36, stackDelay);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
            drive.followTrajectorySequence(toStack);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, stackDelay);
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
            BeepArm.ViperSlideSetPos(16, 36, stackDelay);
            drive.followTrajectorySequence(toCyclePole);
            drive.followTrajectorySequence(front);
            BeepArm.ViperSlideSetPos(6, 6, 250);
            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);
            BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
            drive.followTrajectorySequence(toStack);
            BeepArm.ClawFullClose(750);
            BeepArm.ViperSlideSetPos(16, 36, stackDelay);
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
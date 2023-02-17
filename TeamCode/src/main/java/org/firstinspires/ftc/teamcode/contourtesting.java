package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous (name="contour test", group="Tutorials")

public class contourtesting extends LinearOpMode {
    private OpenCvCamera webcam;

    //Create New Robot based on RobotBase
    RobotBase Beep = new RobotBase();

    // Dont really need to change this.
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Change these values to move the little square/region where we check what color we see
    private static double RegionCenterX = 875; // Distance in pixels from the Left
    private static double RegionCenterY = 340; // Distance in pixels from the top
    private static double RegionWidth = 50;
    private static double RegionHeight = 50;
    public static double Kp = 0.0004;
    public static int clawCenter = 670;
    public static double x = 1;
    public static int pixelMargin = 35;
    private static int currentPos;

    @Override
    public void runOpMode() {
        Beep.init(hardwareMap, this);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Contour_Pole myPipeline;
        webcam.setPipeline(myPipeline = new BBBDetector_Contour_Pole(CAMERA_WIDTH, CAMERA_HEIGHT,clawCenter,pixelMargin ));

        //Webcam streaming on the dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

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

        if (opModeIsActive()) {

            double xStick;
            double yStick=0;
            double error = 0;
            double yerror = 0;

            currentPos = myPipeline.getPosition();
            error = clawCenter - currentPos;


            telemetry.addData("Current Image x", currentPos);
            telemetry.addData("Error", error);
            telemetry.addLine("Starting in 4 sec");
            telemetry.update();

            Beep.BeepArm.ClawFullClose(500);
            Beep.BeepArm.ViperSlideSetPos(26.5,20,0);


            while ( (Math.abs(error) > pixelMargin || Beep.BeepArm.ViperSlideGetPos() < 26.4)  && opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {
           // while (opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {

                xStick = Kp * error;
                xStick = Math.max( -Math.abs(x), Math.min(xStick, Math.abs(x)));

                yStick = Kp * yerror;
                yStick = Math.max( -Math.abs(x), Math.min(yStick, Math.abs(x)));

                Beep.strafe(-xStick, yStick);

                currentPos = myPipeline.getPosition();

                telemetry.addData("Current Image x", currentPos);
                telemetry.addData("Error", error);
                telemetry.addData("Strafe Power", -xStick);
                telemetry.addData("Current Image in Pixels", myPipeline.getWidthpix());
                telemetry.addData("Distance in inches", myPipeline.getWidth());
                telemetry.addData("Viper Height in inches",Beep.BeepArm.ViperSlideGetPos());
                telemetry.update();

                if ( currentPos>1 ) {
                    error = clawCenter - currentPos;
                }
                else{
                    error=0;
                }

                if (Math.abs(error) < pixelMargin ){
                    error=0;
                }
            }
            Beep.strafe(0,0);

            telemetry.addLine("Out of Loop");
            telemetry.update();
           // sleep(1500);
            double lastMove =  myPipeline.getWidth();


            Beep.BeepArm.ViperSlideSetPos(32,12,-1);

            if (lastMove <6 ) {
                Beep.move((double)lastMove, 5.0, -250);
            }
            else{
                Beep.move((double) 6.0, 5.0, -250);
            }

            Beep.BeepArm.ClawFullOpen(500);


            telemetry.addLine("Done Following Pole");
            telemetry.update();

        }
    }
}

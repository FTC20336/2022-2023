package org.firstinspires.ftc.teamcode;

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
    public static double Kp = 0.0005;
    public static double clawCenter = 500;
    public static double x = .25;
    public static int pixelMargin = 100;

    @Override
    public void runOpMode() {
        Beep.init(hardwareMap, this);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Contour_GPT2 myPipeline;
        webcam.setPipeline(myPipeline = new BBBDetector_Contour_GPT2(CAMERA_WIDTH));

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

            double Turnpower;
            double error = 0;


            error = clawCenter - myPipeline.getPosition();
            telemetry.addData("Current Image x", myPipeline.getPosition());
            telemetry.addData("Error", error);
            telemetry.addLine("Starting in 4 sec");
            telemetry.update();
            sleep(4000);
            // while ( (Math.abs(error) > 50) && opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {
            while (opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {

                Turnpower = Kp * error;
                Turnpower = Math.max( -Math.abs(x), Math.min(Turnpower, Math.abs(x)));


                Beep.strafe(-Turnpower);

                telemetry.addData("Current Image x", myPipeline.getPosition());
                telemetry.addData("Error", error);
                telemetry.addData("TurnPower", -Turnpower);
                telemetry.addData("DIstance", myPipeline.getWidth());
                telemetry.update();

                error = clawCenter - myPipeline.getPosition();

                if (Math.abs(error) < 20 ){
                    error=0;
                }
            }
            Beep.strafe(0);
            telemetry.addLine("Done Following Pole");
            telemetry.update();

        }
    }
}

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;

public class AutoVisionAction {

    public void init(LinearOpMode myOp, BBBDetector_Contour_Pole_Cone myPipeline, SampleMecanumDrive drive, RobotArm beepArm) {
        this.myOp = myOp;
        this.myPipeline = myPipeline;
        this.drive = drive;
        BeepArm = beepArm;
    }

    private LinearOpMode myOp;
    BBBDetector_Contour_Pole_Cone myPipeline;
    SampleMecanumDrive drive;
    RobotArm BeepArm;

    // Change these values for the PID and autocorrection
    public static double Kp = 0.0003;
    public static int clawCenter = 690;
    public static double x = .2; // Max power during Strafe
    public static int pixelMargin = 35;
    private static int currentPos;
    private static double currentPosInches;

    private NanoClock clock;
    private double currentAdjustTime;
    public static double adjustTimeLimit = 1;
    private double distAdjust = 0;

    //height before a drop
    public static double preDropH = 7;
    public static double preScanViperTimeout = -1;

    public static long stackDelay = 500;

    public static double stackh = 5;
    public static double stackinc = 1.25;

    public static double preScanDelay = 500;
    
         // Drop Cone function assumes the robot is near a junction
        // We supply the dropping height in inches
        public void dropConeAt(double dropHeight, TrajectorySequence lastTraj) {
            currentPos = myPipeline.getPolePositionPixels();

            BeepArm.ViperSlideSetPos(dropHeight - preDropH, 36, (long) preScanViperTimeout);
            myOp.sleep((long) preScanDelay);

            double xStick;
            double yStick = 0;
            double error = 0;
            double yerror = 0;

            currentPos = myPipeline.getPolePositionPixels();
            error = clawCenter - currentPos;
            distAdjust = 0;

            currentAdjustTime = clock.seconds();

            double lowErrorTimer = clock.seconds();

            while ((Math.abs(error) > pixelMargin || BeepArm.ViperSlideGetPos() < (dropHeight - preDropH - .5)) && (clock.seconds() - currentAdjustTime) < adjustTimeLimit && myOp.opModeIsActive()) {


                xStick = Kp * error;
                xStick = Math.max(-Math.abs(x), Math.min(xStick, Math.abs(x)));

                drive.strafe(-xStick, yStick);
                distAdjust += -xStick;

                currentPos = myPipeline.getPolePositionPixels();

                myOp.telemetry.addData("Current Image x", currentPos);
                myOp.telemetry.addData("Error", error);
                myOp.telemetry.addData("Strafe Power", -xStick);
                myOp.telemetry.addData("Current Image in Pixels", myPipeline.getPoleDistancePixel());
                myOp.telemetry.addData("Distance in inches", String.format("%.2f", myPipeline.getPoleDistanceInches()));
                myOp.telemetry.addData("Viper Height in inches", String.format("%.2f", BeepArm.ViperSlideGetPos()));
                myOp.telemetry.addData("Total Adjust distance", String.format("%.3f", distAdjust));


                if (currentPos > 1) {
                    error = clawCenter - currentPos;
                    myOp.telemetry.addData("Time doing Correction is: ", clock.seconds() - currentAdjustTime);
                } else {
                    // Reset timer
                    // currentAdjustTime = clock.seconds();
                    myOp.telemetry.addLine("Resetting time because not yellow read");
                    myOp.telemetry.update();
                    //  error=0;
                }

                if (Math.abs(error) < pixelMargin) {
                    currentAdjustTime = clock.seconds();
                    myOp.telemetry.addLine("Resetting time because within Margin");
                    error = 0;
                }

                myOp.telemetry.update();
            }
            drive.strafe(0, 0);

            myOp.telemetry.addLine("Out of Loop");
            myOp.telemetry.update();
            // sleep(1500);
            // Using Distance Sensor
            double lastMove = myPipeline.getPoleDistanceInches() + 0.5;


            BeepArm.ViperSlideSetPos(dropHeight, 12, -1);

            TrajectorySequence drop;// = new TrajectorySequence;
            TrajectorySequence back;// = new TrajectorySequence;

            if (lastMove < 6) {
                myOp.telemetry.addData("Moving inches", String.format("%.3f", lastMove));
                myOp.telemetry.update();
                drop = drive.trajectorySequenceBuilder(lastTraj.end())
                        .forward(lastMove,
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                back = drive.trajectorySequenceBuilder(drop.end())
                        .back(lastMove)
                        .build();

                myOp.sleep(250);
            } else {
                myOp.telemetry.addLine("Moving 6 inches");
                myOp.telemetry.update();

                drop = drive.trajectorySequenceBuilder(lastTraj.end())
                        .forward(6,
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                back = drive.trajectorySequenceBuilder(drop.end())
                        .back(6)
                        .build();
                myOp.sleep(250);
            }
            drive.followTrajectorySequence(drop);
            BeepArm.ViperSlideSetPos(dropHeight - 3, 12, 250);

            BeepArm.ClawFullOpen(250);
            drive.followTrajectorySequence(back);

        }




    // Drop Cone function assumes the robot is near a junction
    // We supply the dropping height in inches
    public void dropConeAtNoPID(double dropHeight, TrajectorySequence lastTraj) {
        // Move slide to PreDrop height so we can look at the tip of the pole
        BeepArm.ViperSlideSetPos(dropHeight - preDropH, 36, 1);


        // This is the lateral error in pixels from the center of the claw to the current position of the pole
        double lateralError = clawCenter - currentPos;

        // This is the distance from the pole and the center of the 'cone in the claw' based on the width of the yellow pole
        double lastMove = myPipeline.getPoleDistanceInches();

        // Convert the lateral error from pixels to inches.... This may be need change
        lateralError = lateralError / 25; // Convert pixels to inches for lateral correction

         myOp.telemetry.addData("Lateral Error in Inches", String.format("%.2f", lateralError) );
         myOp.telemetry.addData("Distance in inches", String.format("%.2f", myPipeline.getPoleDistanceInches()));
         myOp.telemetry.update();

         // Move slide up to dropping position to clear the pole
        BeepArm.ViperSlideSetPos(dropHeight, 12, -1);

        TrajectorySequence drop;// = new TrajectorySequence;
        TrajectorySequence back;// = new TrajectorySequence;

        if (lastMove < 6) {
             myOp.telemetry.addData("Moving inches", String.format("%.3f", lastMove));
             myOp.telemetry.update();
            drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    // This needs to be adjusted depending on the orientation
                    .lineToConstantHeading(new Vector2d(lastTraj.end().getX() - lastMove, lastTraj.end().getY() + lateralError),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            back = drive.trajectorySequenceBuilder(drop.end())
                    .back(lastMove)
                    .build();

            myOp.sleep(250);
        } else {
             myOp.telemetry.addLine("Moving 6 inches");
             myOp.telemetry.update();

            drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    .forward(6,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            back = drive.trajectorySequenceBuilder(drop.end())
                    .back(6)
                    .build();
            myOp.sleep(250);
        }
        drive.followTrajectorySequence(drop);
        BeepArm.ViperSlideSetPos(dropHeight - 3, 12, 150);

        BeepArm.ClawFullOpen(250);
        drive.followTrajectorySequence(back);

    }


    // Drop Cone function assumes the robot is near a junction
// We supply the dropping height in inches
    public void pickConeAt(double pickHeight, TrajectorySequence lastTraj) {
        currentPos = myPipeline.getConePositionPixels();

        BeepArm.ViperSlideSetPos(pickHeight, 36, -1);
        // sleep(1000);
        BeepArm.ClawFullOpen(0);

        double xStick;
        double yStick = 0;
        double error = 0;
        double yerror = 0;

        currentPos = myPipeline.getConePositionPixels();
        error = clawCenter - currentPos;
        distAdjust = 0;

        currentAdjustTime = clock.seconds();
        while ((Math.abs(error) > pixelMargin || BeepArm.ViperSlideGetPos() < (pickHeight - preDropH - .5))
                && (clock.seconds() - currentAdjustTime) < adjustTimeLimit
                && myOp.opModeIsActive()) {


            xStick = Kp * error;
            xStick = Math.max(-Math.abs(x), Math.min(xStick, Math.abs(x)));

            drive.strafe(-xStick, yStick);
            distAdjust += -xStick;

            currentPos = myPipeline.getConePositionPixels();

             myOp.telemetry.addData("Current Image x", currentPos);
             myOp.telemetry.addData("Error", error);
             myOp.telemetry.addData("Strafe Power", -xStick);
             myOp.telemetry.addData("Current Image in Pixels", myPipeline.getConeDistancePixel());
            //    myOp.telemetry.addData("Distance in inches", String.format("%.2f" , myPipeline.getConeDistanceInches()) );
             myOp.telemetry.addData("Distance in inches", String.format("%.2f", drive.distanceSensor.getDistance(DistanceUnit.INCH)));
             myOp.telemetry.addData("Viper Height in inches", String.format("%.2f", BeepArm.ViperSlideGetPos()));
             myOp.telemetry.addData("Total Adjust distance", String.format("%.3f", distAdjust));


            if (currentPos > 1) {
                error = clawCenter - currentPos;
                 myOp.telemetry.addData("Time doing Correction is: ", clock.seconds() - currentAdjustTime);
            } else {
                // Reset timer
                currentAdjustTime = clock.seconds();
                 myOp.telemetry.addLine("Resetting time because not yellow read");
                 myOp.telemetry.update();
                // error=0;
            }

            if (Math.abs(error) < pixelMargin) {
                currentAdjustTime = clock.seconds();
                 myOp.telemetry.addLine("Resetting time because within Margin");
                error = 0;
            }

             myOp.telemetry.update();
        }
        drive.strafe(0, 0);

         myOp.telemetry.addLine("Out of Loop");
         myOp.telemetry.update();
        // sleep(1500);
        double lastMove = drive.distanceSensor.getDistance(DistanceUnit.INCH) - 4;
        // double lastMove =  myPipeline.getConeDistanceInches();


        // BeepArm.ViperSlideSetPos(pickHeight,12,-1);

        TrajectorySequence drop;// = new TrajectorySequence;
        TrajectorySequence back;// = new TrajectorySequence;

        if (lastMove < 10) {
             myOp.telemetry.addData("Moving inches", String.format("%.3f", lastMove));
             myOp.telemetry.update();
            drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    .forward(lastMove,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            back = drive.trajectorySequenceBuilder(drop.end())
                    .back(lastMove)
                    .build();

            myOp.sleep(250);
        } else {
             myOp.telemetry.addData("Moving 10 inches, we found: ", String.format("%.3f", lastMove));
             myOp.telemetry.update();

            drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    .forward(6,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            back = drive.trajectorySequenceBuilder(drop.end())
                    .back(6)
                    .build();
            myOp.sleep(250);
        }

        drive.followTrajectorySequence(drop);
        BeepArm.ClawFullClose(400);
        BeepArm.ViperSlideSetPos(pickHeight + 6, 15, 500);

        drive.followTrajectorySequence(back);

      //  return back;

    }
    
}
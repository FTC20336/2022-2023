package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
@Config
public class AutoVisionAction {

    private LinearOpMode myOp;
    private BBBDetector_Contour_Pole_Cone myPipeline;
    private SampleMecanumDrive drive;
    private RobotArm BeepArm;

    public void init(LinearOpMode myOp, BBBDetector_Contour_Pole_Cone myPipeline, SampleMecanumDrive drive, RobotArm beepArm) {
        this.myOp = myOp;
        this.myPipeline = myPipeline;
        this.drive = drive;
        this.BeepArm = beepArm;

        //Initialize clock
        clock = NanoClock.system();
    }

    public  double poleApproachDist = 15; // Pole Center to Center of Robot
    public  double coneApproachDist = 19; // Cone CEnter to CEnter of Robot


    // Change these values for the PID and autocorrection
    public double Kp = 0.0003;
    public int clawCenter = 690;
    public double x = .2; // Max power during Strafe
    public int pixelMargin = 35;
    private int currentPos;
    private double currentPosInches;

    private NanoClock clock;
    private double currentAdjustTime;
    public static double adjustTimeLimit = 1;
    private double distAdjust = 0;

    //height before a drop
    public static double preDropH = 7;
    public static long preScanViperTimeout = -1;

    public static long stackDelay = 500;

    public static double stackh = 5;
    public static double stackinc = 1.25;

    public static double lateralTolerance = 3; // Pole
    public static double coneLateralTolerance = 3.5; // Cone
    public static double poleFwdAftTolerance =4;
    public static double coneFwdAftTolerance =6;

    public static long preScanDelay = 500;
    
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
    // It returns the Pose2D after alignment and dropping
    public Pose2d dropConeAtNoPID(double dropHeight, TrajectorySequence lastTraj) {
        // Move slide to PreDrop height so we can look at the tip of the pole
        BeepArm.ViperSlideSetPos(dropHeight - preDropH, 36, preScanViperTimeout);

        myOp.sleep(preScanDelay);

        double lateralError = myPipeline.getPolePositionFromClawInches();
        double fwdAftError = myPipeline.getPoleDistanceInches();//+BeepArm.getClawToRobotCenter(); // Adding Claw to robot center value to help calculation

        //while ( !myOp.gamepad1.a && myOp.opModeIsActive()){
            // This is the lateral error in pixels from the center of the claw to the current position of the pole
        //lateralError = myPipeline.getPolePositionFromClawInches();

            if (Math.abs(lateralError) > lateralTolerance) {
                myOp.telemetry.addData("Lateral OUTSIDE LIMIT", String.format("%.2f", lateralError));
                lateralError = 0;
            }
            // This is the distance from the pole and the center of the 'cone in the claw' based on the width of the yellow pole
            // Maybe we could read a few values and do the average of let's say 5 values
            //fwdAftError = myPipeline.getPoleDistanceInches();

            // If we measure distance more than (15"-6.75"+4") = 12.25 or less 4.25 " set to move to where the Pole should be
            if (fwdAftError > (poleApproachDist -BeepArm.getClawToRobotCenter() + poleFwdAftTolerance  )
                     || (fwdAftError < (poleApproachDist -BeepArm.getClawToRobotCenter() - poleFwdAftTolerance  )))
            {
                myOp.telemetry.addData("FwdAft OUTSIDE LIMIT", String.format("%.2f", fwdAftError));
                fwdAftError = 6; // (poleApproachDist - BeepArm.getClawToRobotCenter()) ;
            }

            // Convert the lateral error from pixels to inches.... This may be need change
            // y = pixels width for 1" shift at x distance y = 8x2 - 137.73x + 802.52
            // if laterError is too much , we don't correct.. maybe we could try moving left and right.. or back to see move pole

            myOp.telemetry.addData("Lateral Move needed in Inches", String.format("%.2f", lateralError));
            myOp.telemetry.addData("Fwd/Aft Move needed in inches", String.format("%.2f", fwdAftError));
            myOp.telemetry.addData("Fwd/Aft Move needed Pipeline", String.format("%.2f", myPipeline.getPoleDistanceInches()));
            myOp.telemetry.update();
   // }


         // Move slide up to dropping position to clear the pole
        BeepArm.ViperSlideSetPos(dropHeight, 12, -1);

        TrajectorySequence drop;// = new TrajectorySequence;

        // Do calculation to move foward and strafe according to the orientation
        double newPx = lastTraj.end().getX() + Math.cos(lastTraj.end().getHeading()) * fwdAftError
                                                +  Math.sin(lastTraj.end().getHeading()) * lateralError;

        double newPy = lastTraj.end().getY() + Math.sin(lastTraj.end().getHeading()) * fwdAftError
                                                -  Math.cos(lastTraj.end().getHeading()) * lateralError;
/*
        myOp.telemetry.addData("Px Move needed in inches", String.format("%.2f", newPx));
        myOp.telemetry.addData("Py Move needed Pipeline", String.format("%.2f", newPy));
        myOp.telemetry.update();
*/
        drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    // This needs to be adjusted depending on the orientation
                    .lineToConstantHeading(new Vector2d(newPx, newPy),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL *.75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.5))
                    .build();


        // Drive to the Pol
        drive.followTrajectorySequence(drop);

        //Wait a little in case it's wobbly
        myOp.sleep(250);

        // Move slide Down slowly.. and wait xx
        BeepArm.ViperSlideSetPos(dropHeight - 3, 12, 150);

        // Open Claw and wait xx
        BeepArm.ClawFullOpen(250);

        return drop.end();

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

    public Pose2d pickConeAtNoPID(double pickHeight, TrajectorySequence lastTraj) {


        BeepArm.ClawFullOpen(0);
        BeepArm.ViperSlideSetPos(pickHeight, 36, (long) 0);
        // Move slide to PreDrop height so we can look at the tip of the pole
        myOp.sleep((long) preScanDelay);

        //double lateralError = myPipeline.getConePositionPixels()- clawCenter ;
        double lateralError = myPipeline.getConePositionFromClawInches();

        // drive ConeToClaw
        double fwdAftError = drive.ConeToClaw() -3.5;
      //  while (!myOp.gamepad1.a && myOp.opModeIsActive()) {
          //  fwdAftError = drive.ConeToClaw()-3.5;
            lateralError = myPipeline.getConePositionFromClawInches();

            // This is the lateral error in pixels from the center of the claw to the current position of the pole
            //  lateralError = myPipeline.getConePositionPixels() - clawCenter;

            // This is the distance from the pole and the center of the 'cone in the claw' based on the width of the yellow pole
            // Maybe we could read a few values and do the average of let's say 5 values
/*
            if (fwdAftError > 12) {
                fwdAftError = 8;
            } else if (fwdAftError < 3.5) {
                fwdAftError = 6;

*/
            // Convert the lateral error from pixels to inches.... This may be need change
            // y = pixels width for 1" shift at x distance y = 8x2 - 137.73x + 802.52
            // if laterError is too much , we don't correct.. maybe we could try moving left and right.. or back to see move pole
            //  double shiftSlope = 8 * fwdAftError * fwdAftError - 137.73 * fwdAftError + 802.52;
            //    if (shiftSlope != 0) {
            //      lateralError = lateralError / shiftSlope; // Convert pixels to inches for lateral correction
            //  }

            if (Math.abs(lateralError) > coneLateralTolerance) {
                myOp.telemetry.addData("Lateral OUTSIDE LIMIT", String.format("%.2f", lateralError));
                lateralError = 0;
            }
            // This is the distance from the pole and the center of the 'cone in the claw' based on the width of the yellow pole
            // Maybe we could read a few values and do the average of let's say 5 values
            //fwdAftError = myPipeline.getPoleDistanceInches();

            // If we measure distance more than (19"-6.75"+2") = 14.25 or less 10.25 " set to move to where the Pole should be
            if (fwdAftError > (coneApproachDist - BeepArm.getClawToRobotCenter() + coneFwdAftTolerance)
                    || (fwdAftError < (coneApproachDist - BeepArm.getClawToRobotCenter() - coneFwdAftTolerance))) {
                myOp.telemetry.addData("FwdAft OUTSIDE LIMIT", String.format("%.2f", fwdAftError));
                fwdAftError = (coneApproachDist - BeepArm.getClawToRobotCenter());

            }


            myOp.telemetry.addData("Lateral Move needed in Inches", String.format("%.2f", lateralError));
            myOp.telemetry.addData("Fwd/Aft Move needed in inches", String.format("%.2f", fwdAftError));
            myOp.telemetry.update();
            //  }
       // }
       TrajectorySequence pick;// = new TrajectorySequence;

        // Do calculation to move foward and strafe according to the orientation
        double newPx = lastTraj.end().getX() + Math.cos(lastTraj.end().getHeading()) * fwdAftError
                +  Math.sin(lastTraj.end().getHeading()) * lateralError;

        double newPy = lastTraj.end().getY() + Math.sin(lastTraj.end().getHeading()) * fwdAftError
                                     -  Math.cos(lastTraj.end().getHeading()) * lateralError;

        pick = drive.trajectorySequenceBuilder(lastTraj.end())
                // This needs to be adjusted depending on the orientation
                .lineToConstantHeading(new Vector2d(newPx, newPy),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL *.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*.75))
                .build();


        // Pick up COne
        drive.followTrajectorySequence(pick);

        // CLose Claw and wait to make sure we grab it right
        BeepArm.ClawFullClose(450);

        // Wait only xx millisec in case we get stuck in the wall
        BeepArm.ViperSlideSetPos(pickHeight +5, 12, 0);
        myOp.sleep(250);

        // Repose the Robot based on the Robot
      //  drive.setPoseEstimate(new Pose2d(-70,-12, Math.toRadians(180)));

        return pick.end();

    }
}
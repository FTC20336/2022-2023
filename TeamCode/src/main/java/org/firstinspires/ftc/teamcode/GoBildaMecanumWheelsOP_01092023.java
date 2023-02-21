package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Config
@TeleOp(name = "GoBildaMecanumWheelsOP_01092023 USE ME")
public class GoBildaMecanumWheelsOP_01092023 extends LinearOpMode {

    private DcMotor LeftBack;
    private DcMotor LeftFront;
    private DcMotor RightBack;
    private DcMotor RightFront;
    private IMU imu_IMU;
    private DcMotor ViperSlideMotor;
    private Servo claw;

    double Ticks;
    List ViperSlidePresets;
    int SlideSpeed;
    int SlideMove;
    int SlideInitialSpeed;
    int SlideMaxPos;
    int PwrCurve;
    double globalAngle;
    double lastAngle;
    public static double Kp=.011;


    /**
     * Describe this function...
     * Speed is 0 to 1.. 0 to 100% full Speed
     */
    private void Move(double Distance, double Speed, boolean NoStop) {
        double extradist;

        if (NoStop) {
            extradist = 1.5;
        }
        else{
            extradist = 1;
        }

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftBack.setTargetPosition((int) (Distance * 45.27639 * extradist));
        LeftFront.setTargetPosition((int) (Distance * 45.27639* extradist));
        RightBack.setTargetPosition((int) (Distance * 45.27639* extradist));
        RightFront.setTargetPosition((int) (Distance * 45.27639* extradist));

        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftBack.setPower( Speed);
        LeftFront.setPower( Speed);
        RightBack.setPower( Speed);
        RightFront.setPower( Speed);

        if (NoStop) {
            while ( Math.abs(gamepad1.right_stick_x) < 0.2 && Math.abs(LeftBack.getCurrentPosition()) < Math.abs( (int) (Distance * 45.27639 * .99)) ) {
                telemetry.addData("Current Position: ", LeftBack.getCurrentPosition() );
                telemetry.addData("Target Position: ", (int) (Distance * 45.27639 * .99) );
                telemetry.addData("Extradist: ", extradist );
                telemetry.addData("NoStop: ", NoStop );
                telemetry.addLine("Waiting to reach encoder Pos during move");
                telemetry.update();
            }
        }
        else{
            while (Math.abs(gamepad1.right_stick_x) < 0.2 && (opModeIsActive() && ( LeftBack.isBusy() || RightBack.isBusy() || LeftFront.isBusy() || RightFront.isBusy())) ) {
                telemetry.addLine("Waiting For motor during move");
                telemetry.addData("Extradist: ", extradist );
                telemetry.addData("NoStop: ", NoStop );
                telemetry.addLine("Waiting for motor to stop");
                telemetry.update();
            }
        }


        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Describe this function...
     */
    private void SetIMU() {
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    }

    /**
     * Describe this function...
     */
    private void GoToPreset(List PresetList, String Height) {
        if (Height.equals("High")) {
            ViperSlideMotor.setTargetPosition(ViperSlideInchesToTicks(33));
        } else if (Height.equals("Med")) {
            ViperSlideMotor.setTargetPosition(ViperSlideInchesToTicks(24));
        } else if (Height.equals("Low")) {
            ViperSlideMotor.setTargetPosition(ViperSlideInchesToTicks(14));
        } else {
            ViperSlideMotor.setTargetPosition(ViperSlideInchesToTicks(0));
        }
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ViperSlideMotor).setVelocity(SlideSpeed * 3);
    }

    /**
     * Describe this function...
     */
    private int ViperSlideInchesToTicks(int Inches) {
        Ticks = (38.4 / 8.7) * 28 * Inches;
        return (int) Ticks;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double yVel;
        double xVel;
        double rx;
        double denominator;

        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        ViperSlideMotor = hardwareMap.get(DcMotor.class, "ViperSlideMotor");
        claw = hardwareMap.get(Servo.class, "claw");

        // Put initialization blocks here.
        waitForStart();
        SetVars();
        SetMotors();
        SetIMU();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Put loop blocks here.
                yVel = -((Math.exp(Math.abs(PwrCurve * gamepad1.left_stick_y)) - 1) / (Math.exp(PwrCurve) - 1));
                xVel = (Math.exp(Math.abs(PwrCurve * gamepad1.left_stick_x)) - 1) / (Math.exp(PwrCurve) - 1);
                if (gamepad1.left_stick_y <= 0) {
                    yVel = -yVel;
                }
                if (gamepad1.left_stick_x <= 0) {
                    xVel = -xVel;
                }
                rx = (Math.exp(Math.abs(PwrCurve * gamepad1.right_stick_x)) - 1) / (Math.exp(PwrCurve) - 1);
                if (gamepad1.right_stick_x <= 0) {
                    rx = -rx;
                }
                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    xVel = xVel / 4;
                    yVel = yVel / 4;
                    rx = rx / 4;
                    SlideSpeed = SlideInitialSpeed / 4;
                }
                if (!gamepad1.right_bumper && !gamepad2.right_bumper) {
                    SlideSpeed = SlideInitialSpeed;
                }
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(yVel), Math.abs(xVel), Math.abs(rx))), 1));
                LeftFront.setPower((yVel + xVel + rx) / denominator);
                LeftBack.setPower(((yVel - xVel) + rx) / denominator);
                RightFront.setPower(((yVel - xVel) - rx) / denominator);
                RightBack.setPower(((yVel + xVel) - rx) / denominator);
                if (claw.getPosition() > 0.194) {
                    claw.setPosition(0.194);
                } else {
                    if (gamepad1.x || gamepad2.x) {
                        claw.setPosition(0);
                    }
                    if (gamepad1.y || gamepad2.y) {
                        claw.setPosition(claw.getPosition() + 0.02);
                    }
                }
                if (ViperSlideMotor.getCurrentPosition() < 0) {
                    ViperSlideMotor.setTargetPosition(0);
                    MoveViperSlide(false);
                } else if (ViperSlideMotor.getCurrentPosition() > (SlideMaxPos)) {
                    ViperSlideMotor.setTargetPosition(SlideMaxPos);
                    MoveViperSlide(false);
                } else {
                    if (gamepad1.a || gamepad2.a) {
                        ViperSlideMotor.setTargetPosition(ViperSlideMotor.getCurrentPosition() + SlideSpeed);
                        MoveViperSlide(true);
                    }
                    if (gamepad1.b || gamepad2.b  ) {
                        ViperSlideMotor.setTargetPosition(ViperSlideMotor.getCurrentPosition() - SlideSpeed);
                        MoveViperSlide(true);
                    }
                    if (!gamepad1.a && !gamepad1.b && !gamepad2.a && !gamepad2.b) {
                        if (SlideMove == 1) {
                            ViperSlideMotor.setTargetPosition(ViperSlideMotor.getCurrentPosition());
                            SlideMove = 0;
                        }
                    }
                    if (gamepad1.start) {
                        ViperSlideMotor.setTargetPosition(SlideMaxPos);
                        MoveViperSlide(false);
                    }
                }
                if (gamepad2.dpad_up) {
                    ViperSlideMotor.setTargetPosition(SlideMaxPos);
                    MoveViperSlide(true);
                    ViperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (gamepad2.dpad_down) {
                    ViperSlideMotor.setTargetPosition(ViperSlideMotor.getCurrentPosition() - SlideSpeed);
                    MoveViperSlide(true);
                    ViperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (gamepad2.left_bumper) {
                    GoToPreset(ViperSlidePresets, "High");
                }
                if (gamepad2.back) {
                    GoToPreset(ViperSlidePresets, "Med");
                }
                if (gamepad2.start) {
                    GoToPreset(ViperSlidePresets, "Low");
                }
                if (gamepad2.right_stick_button) {
                    imu_IMU.resetYaw();
                }
                if (gamepad1.dpad_left) {
                    TurnLeft180new(.75,180,false);
                }
                if (gamepad1.dpad_right) {
                    TurnLeft180new(-0.75, 180, false);
                }
                if (gamepad2.dpad_left) {
                    AlignHeading(0, .9);
                }
                if (gamepad2.dpad_right) {
                    AlignHeading(180, .9);
                }
                telemetry.addLine("Version 11323-1914");
                telemetry.addData("Robot Heading", String.format("%.3f" ,imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
                telemetry.addData("Ticks", Ticks);
                telemetry.addData("Servo Pos", claw.getPosition());
                telemetry.addData("YVel", yVel);
                telemetry.addData("XVel", xVel);
                telemetry.addData("Rotation", rx);
                telemetry.addData("Slide Position", ViperSlideMotor.getCurrentPosition());
                telemetry.addData("SlideSpeed", ((DcMotorEx) ViperSlideMotor).getVelocity());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void MoveViperSlide(boolean SlideMove_) {
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ViperSlideMotor).setVelocity(SlideSpeed);
        if (SlideMove_ == true) {
            SlideMove = 1;
        }
    }

    /**
     * Describe this function...
     */
    private void SetMotors() {
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private void SetVars() {
        int SlidePos;
        int MaxPower;

        // Presets: 0 = High, 1 = Med, 2 = Low
        // Put run blocks here.
        SlidePos = 0;
        SlideSpeed = 2000;
        SlideInitialSpeed = 2000;
        SlideMaxPos = 4400;
        SlideMove = 0;
        // PwerCurve Value from .001 to 4.. NEVER 0
        PwrCurve = 1;
        MaxPower = 1;
        imu_IMU.resetYaw();
    }

    /**
     * Describe this function...
     * -1 for Right Turn going back to grab a cone
     * +1 for left turn and dropping a cone
     */
    private void TurnLeft180(int x) {
        double Kp;
        int error;
        double YawAngle;
        double Turnpower;


        if (x < 1) {
            GoToPreset(ViperSlidePresets, "Bottom");
            Move(-15, 0.9, false);
        } else {
            GoToPreset(ViperSlidePresets, "Low");
            Move(-7, 0.9, false);
            GoToPreset(ViperSlidePresets, "High");
        }
        Kp = 0.011;
        error = 99;

        imu_IMU.resetYaw();


        while (Math.abs(error) > 3 && opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {
            YawAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = (int) (180 - Math.abs(YawAngle));
            if (YawAngle > 2) {
                error = -error;
            }
            Turnpower = Kp * error;
            Turnpower = Math.min(Math.max(Turnpower, -1), 1);

            LeftBack.setPower(Turnpower * Math.abs(x));
            LeftFront.setPower(Turnpower * Math.abs(x));
            RightFront.setPower(-(Turnpower * Math.abs(x)));
            RightBack.setPower(-(Turnpower * Math.abs(x)));

            telemetry.addData("YAW", YawAngle);
            telemetry.addData("Error", error);
            telemetry.addData("TurnPower", Turnpower);
            telemetry.update();
        }
        //TO be safe Set power motors to 0
        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        if (x < 1) {
            GoToPreset(ViperSlidePresets, "Bottom");
            Move(6.5, 0.8, false);

        } else {
            GoToPreset(ViperSlidePresets, "High");
            Move(15, 0.8, false);

        }
    }
    /**
     * Describe this function...
     * x: +1 to go drop a cone / - 1 to return and get a new cone
     * turnangle: how many degrees to turn.. + left turn /- right turn
     * UseProvidedAngle: turn 'turnangle' degrees  if True.. if not go to 0 and 180 base of imu.. careful if this is not set properly
     */

    private void TurnLeft180new( double x , double turnangle, boolean UseProvidedAngle) {

        double error;
        double Turnpower;
        double TurnPrecision = 1;

        if (x < 0) {
            Move(-12, .7, true);
            GoToPreset(ViperSlidePresets, "Bottom");
           if (!UseProvidedAngle){
                turnangle = 180 - imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)  ; //0 is the target heading
                if (turnangle > 180){
                    turnangle = turnangle -360; // Turn Right.. it's a bit quicker
                }
           }
         //Have a Cone.. and going back to turn and drop it
        } else {
            GoToPreset(ViperSlidePresets, "Low");
            Move(-7, .7, true);
            GoToPreset(ViperSlidePresets, "High");
            if (!UseProvidedAngle) {
                turnangle = -imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // Will turn Left or right.. will take shortness way
            }
        }
        Kp = 0.011;

        resetAngle();
        error = turnangle - getAngle();

        while ( (Math.abs(error) > TurnPrecision) && opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {

            Turnpower = Kp * error ;
            Turnpower = Math.min(Math.max(Turnpower, -Math.abs(x)), Math.abs(x));

            LeftBack.setPower(-Turnpower );
            LeftFront.setPower(-Turnpower );
            RightFront.setPower(Turnpower );
            RightBack.setPower( Turnpower );

            telemetry.addData("TurnAngle", turnangle);
            telemetry.addData("Globalangle", globalAngle);
            telemetry.addData("Error", error);
            telemetry.addData("TurnPower", Turnpower);
            telemetry.update();
            error = turnangle - getAngle();
        }
       // LeftBack.setPower(0);
       // LeftFront.setPower(0);
       // RightFront.setPower(0);
       // RightBack.setPower(0);

        if (x < 0) {
            GoToPreset(ViperSlidePresets, "Bottom");
            Move(6.5, .6, false);

        } else {
            GoToPreset(ViperSlidePresets, "High");
            Move(12, .45, false);
        }
    }


    private void AlignHeading(double  headingtarget, double x) {

        double error;
        double turnangle;
        double Turnpower;
        double TurnPrecision = 1;


        turnangle = headingtarget - imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)  ;
        if (turnangle > 180) {
            turnangle = turnangle - 360; // Turn Right.. it's a bit quicker
        }
        if (turnangle < -180) {
            turnangle = turnangle + 360; // Turn Right.. it's a bit quicker
        }

        resetAngle();
        error = turnangle - getAngle();

        while ( (Math.abs(error) > TurnPrecision) && opModeIsActive() && Math.abs(gamepad1.right_stick_x) < 0.2) {

            Turnpower = Kp * error ;
            Turnpower = Math.min(Math.max(Turnpower, -Math.abs(x)), Math.abs(x));

            LeftBack.setPower(-Turnpower );
            LeftFront.setPower(-Turnpower );
            RightFront.setPower(Turnpower );
            RightBack.setPower( Turnpower );

            telemetry.addData("TurnAngle", turnangle);
            telemetry.addData("IMU Heading", imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Globalangle", globalAngle);
            telemetry.addData("Error", error);
            telemetry.addData("TurnPower", Turnpower);
            telemetry.update();
            error = turnangle - getAngle();
        }
        LeftBack.setPower(0);
        LeftFront.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        double current_angle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double deltaAngle = current_angle- lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = current_angle;

        return globalAngle;
    }
}
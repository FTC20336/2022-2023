package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotBase;

@Config
@TeleOp(name = "GoBildaMecanumWheelsOP_12292022_JAVA")
public class GoBildaMecanumWheelsOP_122922_JAVA extends LinearOpMode {

    private IMU imu_IMU;
    private DcMotor ViperSlideMotor;
    private DigitalChannel green;
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;
    private Servo claw;

    int SlideSpeed;
    double Ticks;
    List ViperSlidePresets;
    int SlideMove;
    int SlideInitialSpeed;
    int SlideMaxPos;
    int PwrCurve;

    public static double kp = .1;
    public static double ki = 0;
    public static double kd = 0;

    static private double COUNT_PER_360_ROTATE = 3990;

    LinearOpMode MyOp = null;


    /**
     * Describe this function...
     */
    private void SetIMU() {
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    }

    public void rotate2(double angle, double speed, long timeout) {
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        angle *= 2;

        RightBack.setTargetPosition((int) ((angle / 360) * COUNT_PER_360_ROTATE));
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftBack.setTargetPosition((int) ((-angle / 360) * COUNT_PER_360_ROTATE));
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setTargetPosition((int) ((angle / 360) * COUNT_PER_360_ROTATE));
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftFront.setTargetPosition((int) ((-angle / 360) * COUNT_PER_360_ROTATE));
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        YawPitchRollAngles initHeading ;

        initHeading = imu_IMU.getRobotYawPitchRollAngles();
        double yaw = initHeading.getYaw(AngleUnit.DEGREES);
        double dt = initHeading.getAcquisitionTime();
        double prevTime = dt;



        double previouserror = 0;
        double integral = 0;
        double error = angle - yaw;
        double propertional;
        double derivative;
        double output;

        while (error > 0.5) {
            initHeading = imu_IMU.getRobotYawPitchRollAngles();
            yaw = initHeading.getYaw(AngleUnit.DEGREES);
            dt = initHeading.getAcquisitionTime() - prevTime;

            error = angle - yaw;
            propertional = error;
            integral += error * dt;
            derivative = (error - previouserror) / dt;

            output = kp * propertional + ki * integral + kd * derivative;

            RightBack.setPower(output * speed);
            LeftBack.setPower(output * speed);
            RightFront.setPower(output * speed);
            LeftFront.setPower(output * speed);

            previouserror = error;
        }

        RightBack.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        LeftFront.setPower(0);

        while (MyOp.opModeIsActive() && ( LeftBack.isBusy() || RightBack.isBusy() || LeftFront.isBusy() || RightFront.isBusy()))
        {}

        MyOp.sleep(Math.abs(timeout));


    }

    /**
     * Describe this function...
     */
    private double ViperSlideInchesToTicks(double Inches) {
        Ticks = (38.4 / 8.7) * 28 * Inches;
        return Ticks;
    }

    /**
     * Describe this function...
     */
    private void GoToPreset(List PresetList, String Height) {
        if (Height.equals("High")) {
            ViperSlideMotor.setTargetPosition(33);
        } else if (Height.equals("Med")) {
            ViperSlideMotor.setTargetPosition(24);
        } else {
            ViperSlideMotor.setTargetPosition(14);
        }
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ViperSlideMotor).setVelocity(SlideSpeed * 3);
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

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        ViperSlideMotor = hardwareMap.get(DcMotor.class, "ViperSlideMotor");
        green = hardwareMap.get(DigitalChannel.class, "green");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        claw = hardwareMap.get(Servo.class, "claw");

        // Put initialization blocks here.
        waitForStart();
        SetVars();
        SetMotors();
        SetIMU();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                green.setState(false);
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
                    xVel = xVel / 5;
                    yVel = yVel / 5;
                    rx = rx / 5;
                    SlideSpeed = SlideInitialSpeed / 5;
                    green.setState(true);
                }
                if (!gamepad1.right_bumper && !gamepad2.right_bumper) {
                    SlideSpeed = SlideInitialSpeed;
                    green.setState(false);
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
                } else if (ViperSlideMotor.getCurrentPosition() > SlideMaxPos) {
                    ViperSlideMotor.setTargetPosition(SlideMaxPos);
                    MoveViperSlide(false);
                } else {
                    if (gamepad1.a || gamepad2.a) {
                        ViperSlideMotor.setTargetPosition(ViperSlideMotor.getCurrentPosition() + SlideSpeed);
                        MoveViperSlide(true);
                    }
                    if (gamepad1.b || gamepad2.b) {
                        ViperSlideMotor.setTargetPosition(ViperSlideMotor.getCurrentPosition() + SlideSpeed);
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
                if (gamepad1.dpad_left) {
                    rotate2(180, 1, 0);
                }
                telemetry.addData("Robot Orientation", imu_IMU.getRobotYawPitchRollAngles());
                telemetry.addData("Ticks", Ticks);
                telemetry.addData("Servo Pos", claw.getPosition());
                telemetry.addData("Pwr Curve", PwrCurve);
                telemetry.addData("YPos", gamepad1.left_stick_y);
                telemetry.addData("YVel", yVel);
                telemetry.addData("XVel", xVel);
                telemetry.addData("Rotation", rx);
                telemetry.addData("Slide Position", ViperSlideMotor.getCurrentPosition());
                telemetry.addData("SlideSpeed", ((DcMotorEx) ViperSlideMotor).getVelocity());
                telemetry.addData("LEDGreen", green.getState());
                telemetry.update();
            }
        }
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
        green.setMode(DigitalChannel.Mode.OUTPUT);
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
    private void MoveViperSlide(boolean SlideMove_) {
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ViperSlideMotor).setVelocity(SlideSpeed);
        if (SlideMove_ == true) {
            SlideMove = 1;
        }
    }
}

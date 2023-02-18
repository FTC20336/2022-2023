package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class RobotBase {

  //May have to make these public
  private IMU imu_IMU;
  public DcMotorEx LeftFront;
  public DcMotorEx LeftBack;
  public DcMotorEx RightFront;
  public DcMotorEx RightBack;
  public DistanceSensor distanceSensor;
  public RobotArm BeepArm= new RobotArm();

  static private double PI = 3.141592;
  static private double CIRCUMFERENCE = 96 / 25.4 * PI;
  /*static private double GEAR_3_RATIO = 2.89;
  static private double GEAR_4_RATIO = 3.61;
  static private double GEAR_5_RATIO = 5.23;*/
  static private double MOTOR_GEAR_RATIO = 19.2;
  static private double COUNTS_PER_IN_DRIVE = 28 * MOTOR_GEAR_RATIO / CIRCUMFERENCE;
  static private double STRAFE_FACTOR = 1.12;

  // How Many Encoder Tick for a 360 turn with all wheel turning
  static private double COUNT_PER_360_ROTATE = 3990;
  // How Many Encoder Tick per second for 1 turn in 1 second with all wheel turning
  static private double COUNT_PER_360_ROTATE_SPEED = 11.5;

  public static double kp = .1;
  public static double ki = 0;
  public static double kd = 0;

  // Local OpMode members
  HardwareMap hwMap = null;
  LinearOpMode MyOp = null;

  // Constructor - leave this blank for now
  public RobotBase() {

  }

  private void SetIMU() {
    // Initializes the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
  }

  // Distance in inches
  // Speed in inches/sec
  public void move(double distance, double speed, long timeout) {
    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    RightBack.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightBack.setVelocity(COUNTS_PER_IN_DRIVE * speed); // Set Velocity is in Ticks per Second

    LeftFront.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftFront.setVelocity(COUNTS_PER_IN_DRIVE * speed);

    RightFront.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setVelocity(COUNTS_PER_IN_DRIVE * speed);


    LeftBack.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftBack.setVelocity(COUNTS_PER_IN_DRIVE * speed);


    while (MyOp.opModeIsActive() && ( LeftBack.isBusy() || RightBack.isBusy() || LeftFront.isBusy() || RightFront.isBusy()))
    {}

    MyOp.sleep(Math.abs(timeout));

  }

  // Rotate around center of Robot
  // Degrees and degrees per sec
  public void rotate(double angle, double speed, long timeout) {

    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    RightBack.setTargetPosition((int) ((angle / 360) * COUNT_PER_360_ROTATE));
    RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightBack.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    LeftBack.setTargetPosition((int) ((-angle / 360) * COUNT_PER_360_ROTATE));
    LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftBack.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    RightFront.setTargetPosition((int) ((angle / 360) * COUNT_PER_360_ROTATE));
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    LeftFront.setTargetPosition((int) ((-angle / 360) * COUNT_PER_360_ROTATE));
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftFront.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    while (MyOp.opModeIsActive() && ( LeftBack.isBusy() || RightBack.isBusy() || LeftFront.isBusy() || RightFront.isBusy()))
    {}

    MyOp.sleep(Math.abs(timeout));


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


  public void strafe(double distance, double angle, double speed, long timeout) {


    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Speeds
    double y = Math.cos( Math.toRadians(angle)) * speed;
    double x = Math.sin(Math.toRadians(angle)) * speed;

    //Position
    double dy = Math.cos(Math.toRadians(angle)) * distance;
    double dx = Math.sin(Math.toRadians(angle)) * distance * STRAFE_FACTOR;

    RightBack.setTargetPosition((int) ((dy + dx) * COUNTS_PER_IN_DRIVE)); // Set Position is in Ticks per Second
    LeftFront.setTargetPosition((int) ((dy + dx) * COUNTS_PER_IN_DRIVE));
    RightFront.setTargetPosition((int) ((dy - dx) * COUNTS_PER_IN_DRIVE));
    LeftBack.setTargetPosition((int) ((dy - dx) * COUNTS_PER_IN_DRIVE));

    RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    RightBack.setVelocity((y + x) * COUNTS_PER_IN_DRIVE); // Set Velocity is in Ticks per Second
    LeftFront.setVelocity((y + x) * COUNTS_PER_IN_DRIVE);
    RightFront.setVelocity((y - x) * COUNTS_PER_IN_DRIVE);
    LeftBack.setVelocity((y - x) * COUNTS_PER_IN_DRIVE);


    while (MyOp.opModeIsActive() && ( LeftBack.isBusy() || RightBack.isBusy() || LeftFront.isBusy() || RightFront.isBusy()))
    {}

    MyOp.sleep(Math.abs(timeout));



  }


  public void strafe(double xStick, double yStick) {

    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    RightBack.setPower( (yStick + xStick) );
    LeftFront.setPower( (yStick + xStick) );
    RightFront.setPower( (yStick - xStick) );
    LeftBack.setPower( (yStick - xStick) );

  }

//Dont use me yet
  public void strafe2(double distance, double angle, double speed, double rotateangle, long timeout) {


    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Speeds
    double y = Math.cos( Math.toRadians(angle)) * speed;
    double x = Math.sin(Math.toRadians(angle)) * speed * STRAFE_FACTOR;

    double xrotated = x;//  - y ;
    double yrotated = y ;// - y ;
    double rxs = rotateangle ;// * speed;

    //Position
    double dy = Math.cos(Math.toRadians(angle)) * distance;
    double dx = Math.sin(Math.toRadians(angle)) * distance * STRAFE_FACTOR;

    double xrotateds = dx ;//- dy ;
    double yrotateds = dy ;// - dy ;
    double rx = rotateangle ;// * speed;

    RightBack.setTargetPosition((int) ((yrotateds + xrotateds - rx) * COUNTS_PER_IN_DRIVE*STRAFE_FACTOR)); // Set Velocity is in Ticks per Second
    LeftFront.setTargetPosition((int) ((yrotateds + xrotateds + rx) * COUNTS_PER_IN_DRIVE*STRAFE_FACTOR));
    RightFront.setTargetPosition((int) ((yrotateds - xrotateds - rx) * COUNTS_PER_IN_DRIVE*STRAFE_FACTOR));
    LeftBack.setTargetPosition((int) ((yrotateds - xrotateds + rx) * COUNTS_PER_IN_DRIVE*STRAFE_FACTOR));

    RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    RightBack.setVelocity((yrotated + xrotated - rxs ) * COUNTS_PER_IN_DRIVE); // Set Velocity is in Ticks per Second
    LeftFront.setVelocity((yrotated + xrotated + rxs ) * COUNTS_PER_IN_DRIVE);
    RightFront.setVelocity((yrotated - xrotated - rxs) * COUNTS_PER_IN_DRIVE);
    LeftBack.setVelocity((yrotated - xrotated + rxs) * COUNTS_PER_IN_DRIVE);


    while (MyOp.opModeIsActive() && ( LeftBack.isBusy() || RightBack.isBusy() || LeftFront.isBusy() || RightFront.isBusy()))
    {}

    MyOp.sleep(Math.abs(timeout));



  }

  /**
   *
   */
  /* Initialize standard Hardware interfaces */
  public void init(HardwareMap ahwMap, LinearOpMode MyOpin) {
    // Save reference to Hardware map

    BeepArm.init(ahwMap, MyOpin);
    hwMap = ahwMap;
    MyOp = MyOpin;

    // Define and Initialize Motors.  Assign Names that match the setup on the DriverHub

    LeftBack = hwMap.get(DcMotorEx.class, "LeftBack");
    RightBack = hwMap.get(DcMotorEx.class, "RightBack");
    LeftFront = hwMap.get(DcMotorEx.class, "LeftFront");
    RightFront = hwMap.get(DcMotorEx.class, "RightFront");
    RightFront = hwMap.get(DcMotorEx.class, "RightFront");

    distanceSensor = hwMap.get(DistanceSensor.class, "dist");

    imu_IMU = hwMap.get(IMU.class, "imu");
    SetIMU();


    // Reverse one of the drive motors.
    RightFront.setDirection(DcMotorEx.Direction.FORWARD);
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  }
}


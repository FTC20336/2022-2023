package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotBase {

  //May have to make these public
  public DcMotorEx MotorRight;
  public DcMotorEx MotorLeft;
  public DcMotorEx frontRightWheel;
  public DcMotorEx frontLeftMotor;
  public DcMotorEx TopWheel;
  public RobotArm BeepArm= new RobotArm();

  static private double PI = 3.141592;
  static private double CIRCUMFERENCE = 76 / 25.4 * PI;
  static private double GEAR_3_RATIO = 2.89;
  static private double GEAR_4_RATIO = 3.61;
  static private double GEAR_5_RATIO = 5.23;
  static private double COUNTS_PER_IN_DRIVE = 28 * GEAR_3_RATIO * GEAR_4_RATIO * GEAR_3_RATIO / CIRCUMFERENCE;

  // How Many Encoder Tick for a 360 turn with all wheel turning
  static private double COUNT_PER_360_ROTATE = 6300;
  // How Many Encoder Tick per second for 1 turn in 1 second with all wheel turning
  static private double COUNT_PER_360_ROTATE_SPEED = 25.5;

  // Local OpMode members
  HardwareMap hwMap = null;
  LinearOpMode MyOp = null;

  // Constructor - leave this blank for now
  public RobotBase() {

  }

  public void DuckWheelRed(){
    DuckWheel(-.5);
  }

  public void DuckWheelBlue(){
    DuckWheel(.5);
  }


  // Direction.. -1 Spins Wheel for Red Position
//              1 Spins Wheel for Blue Position
  private void DuckWheel(double Direction){
    TopWheel.setPower (Direction);
  }

  public void DuckWheelStop(){
    TopWheel.setPower (0);
  }

  // Distance in inches
  // Speed in inches/sec
  public void move(double distance, double speed, long timeout) {
    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    MotorRight.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorRight.setVelocity(COUNTS_PER_IN_DRIVE * speed); // Set Velocity is in Ticks per Second

    frontLeftMotor.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeftMotor.setVelocity(COUNTS_PER_IN_DRIVE * speed);

    frontRightWheel.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRightWheel.setVelocity(COUNTS_PER_IN_DRIVE * speed);


    MotorLeft.setTargetPosition((int) (distance * COUNTS_PER_IN_DRIVE));
    MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorLeft.setVelocity(COUNTS_PER_IN_DRIVE * speed);


    while (MyOp.opModeIsActive() && ( MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()))
    {}

    MyOp.sleep(Math.abs(timeout));

  }

  // Rotate around center of Robot
  // Degrees and degrees per sec
  public void rotate(double angle, double speed, long timeout) {

    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    MotorRight.setTargetPosition((int) ((angle / 360) * COUNT_PER_360_ROTATE));
    MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorRight.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    MotorLeft.setTargetPosition((int) ((-angle / 360) * COUNT_PER_360_ROTATE));
    MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorLeft.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    frontRightWheel.setTargetPosition((int) ((angle / 360) * COUNT_PER_360_ROTATE));
    frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRightWheel.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    frontLeftMotor.setTargetPosition((int) ((-angle / 360) * COUNT_PER_360_ROTATE));
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeftMotor.setVelocity(speed * COUNT_PER_360_ROTATE_SPEED);

    while (MyOp.opModeIsActive() && ( MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()))
    {}

    MyOp.sleep(Math.abs(timeout));


  }


  public void strafe(double distance, double angle, double speed, long timeout) {


    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    double y = Math.cos(angle * PI / 180) * speed;
    double x = Math.sin(angle * PI / 180) * speed;
    double dy = Math.cos(angle * PI / 180) * distance;
    double dx = Math.sin(angle * PI / 180) * distance;

    MotorRight.setTargetPosition((int) ((dy + dx) * COUNTS_PER_IN_DRIVE)); // Set Velocity is in Ticks per Second
    frontLeftMotor.setTargetPosition((int) ((dy + dx) * COUNTS_PER_IN_DRIVE));
    frontRightWheel.setTargetPosition((int) ((dy - dx) * COUNTS_PER_IN_DRIVE));
    MotorLeft.setTargetPosition((int) ((dy - dx) * COUNTS_PER_IN_DRIVE));

    MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    MotorRight.setVelocity((y + x) * COUNTS_PER_IN_DRIVE); // Set Velocity is in Ticks per Second
    frontLeftMotor.setVelocity((y + x) * COUNTS_PER_IN_DRIVE);
    frontRightWheel.setVelocity((y - x) * COUNTS_PER_IN_DRIVE);
    MotorLeft.setVelocity((y - x) * COUNTS_PER_IN_DRIVE);


    while (MyOp.opModeIsActive() && ( MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()))
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

    MotorLeft = hwMap.get(DcMotorEx.class, "MotorLeft");
    MotorRight = hwMap.get(DcMotorEx.class, "MotorRight");
    frontLeftMotor = hwMap.get(DcMotorEx.class, "frontLeftMotor");
    frontRightWheel = hwMap.get(DcMotorEx.class, "frontRightWheel");
    TopWheel = hwMap.get(DcMotorEx.class, "topwheelmotor");


    // Reverse one of the drive motors.
    frontRightWheel.setDirection(DcMotorEx.Direction.FORWARD);
    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    MotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    MotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  }
}


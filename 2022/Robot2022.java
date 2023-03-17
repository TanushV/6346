// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Drive Controller (1st Controller Plugged In):
 * Left Stick: forward, backward, rotation drive
 * Right Stick: left and right strafe
 * Start button: invert drive
 * Left bumper: left climber up
 * Right bumper: right climber up
 * Left trigger: left climber down
 * Right trigger: right climber down
 * D-Pad up: increase drive sensitivity
 * D-Pad down: decrease drive sensitivity
 * 
 * Shoot Controller (2nd Controller Plugged In):
 * Left stick: intake and outtake
 * Right stick: uptake and downtake
 * Left bumper: shoot static (set to 60% for low goal)
 * Right bumper: shoot static (by SmartDashboard, default 60%)
 * Left trigger: reverse shooter (set to -20%)
 * Right trigger: shoot variable
 * A button: auto alignment of shooter
 */

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {
  private MecanumDrive m_robotDrive;
  private XboxController driveController;
  private XboxController shootController;
  private boolean invertedDrive = false;
  
  private CANSparkMax frontLeft = new CANSparkMax(7, MotorType.kBrushed);
  private CANSparkMax rearLeft = new CANSparkMax(6, MotorType.kBrushed);
  private CANSparkMax frontRight = new CANSparkMax(4, MotorType.kBrushed);
  private CANSparkMax rearRight = new CANSparkMax(5, MotorType.kBrushed);

  private CANSparkMax shooterLeft = new CANSparkMax(8, MotorType.kBrushless);
  private CANSparkMax shooterRight = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax climb1 = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax climb2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax index1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax index2 = new CANSparkMax(10, MotorType.kBrushless);
  // private CANSparkMax pivot = new CANSparkMax(11, MotorType.kBrushless);

  // private CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushed);
  // private CANSparkMax uptake = new CANSparkMax(10, MotorType.kBrushed);
  private Spark intake = new Spark(1);
  private Spark uptake = new Spark(0);
  
  // private Spark led = new Spark(2);

  private double shooterSpeed;
  private double sens;

  private Timer timer = new Timer();

  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry limelightLED = limelight.getEntry("ledMode");
  private NetworkTableEntry pipeline = limelight.getEntry("pipeline");
  private NetworkTableEntry tx = limelight.getEntry("tx");
  private NetworkTableEntry ty = limelight.getEntry("ty");

  @Override
  public void robotInit() {

    // Invert the right side motors.
    frontLeft.setInverted(true);
    frontRight.setInverted(false);
    rearLeft.setInverted(true);
    rearRight.setInverted(false);
    shooterLeft.setInverted(false);
    index1.setInverted(true);
    intake.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_robotDrive.setDeadband(0.2);

    driveController = new XboxController(0);
    shootController = new XboxController(1);

    shooterRight.follow(shooterLeft, true);
    index2.follow(index1, true);
    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);

    shooterSpeed = 0.89;
    sens = 1.0;
 
    limelightLED.setNumber(1);
    SmartDashboard.setDefaultBoolean("Shooter Override", false);

  }

  //---------------------------------------------------------------------------------------------------------

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    limelightLED.setNumber(3);
    pipeline.setNumber(1);
  }

  @Override
  public void autonomousPeriodic() {
    //drop intake
    if (timer.get() < 2 ) {
      intake.set(0.25); 
    } else if (timer.get() > 2.1 && timer.get() < 2.2) {
      intake.stopMotor();
    }    
    
    //drive back a little
    if (timer.get() > 2.2 && timer.get() < 3.2) {
      drive(0.3, 0.3, 0.3, 0.3);
    } else if (timer.get() > 3.3 && timer.get() < 3.4) {
      stopDrive();
    }

    //limelight align and engage shooter
    if (timer.get() > 3.5 && timer.get() < 5) {
      autoAlignTurret();
      shooterLeft.set(0.85);
    } else if (timer.get() > 5.1 && timer.get() < 7) {
      autoDistance();
    }

    //uptake ball
    if (timer.get() > 7.1 && timer.get() < 7.9) {
      uptake.set(-0.5);
    } else if (timer.get() > 8 && timer.get() < 8.1) {
      uptake.stopMotor();
    }

    //engage intake and move back
    if (timer.get() > 8.2 && timer.get() < 9.5) {
      intake.set(-0.75);
      drive(0.3, 0.3, 0.3, 0.3);
    } else if (timer.get() > 9.6){
      stopDrive();
    }

    //limelight align again
    if (timer.get() > 9.7 && timer.get() < 11.5) {
      autoAlignTurret();
    } else if (timer.get() > 11.6 && timer.get() < 13) {
      autoDistance();
    }

    //engage uptake again
    if (timer.get() > 13.1) {
      uptake.set(-0.5);
    }

    //stop shooter
    if (timer.get() > 14.9) {
      shooterLeft.stopMotor();
    }

  }

  //------------------------------------------------------------------------------------------------

  @Override
  public void teleopPeriodic() {

    double ySpeed;
    double xSpeed;
    double zRot;

    // Inverts drive direction if start button is pressed
    if (driveController.getStartButtonPressed()) invertedDrive = !invertedDrive;
    
    // Increases and decreases the joystick sensitivity when up and down on the DPad is pressed
    switch (driveController.getPOV()) {
        case 0:
          if (sens < 1.0) sens += 0.05;
          // System.out.println("Increased sens to " + sens);
          break;
        case 180:
          if (sens > 0.1) sens -= 0.05;
          // System.out.println("Decreased sens to " + sens);
          break;
          
        default:
          sens = SmartDashboard.getNumber("Sensitivity", sens);
          break;
      }

    // Assigns joystick direction to movement based on inverted state
    // Changes the LED lights to match inverted state 
    if (invertedDrive) {
      ySpeed = driveController.getRawAxis(1);
      xSpeed = -driveController.getRawAxis(4);
      zRot = driveController.getRawAxis(0);
    } else {
      ySpeed = -driveController.getRawAxis(1);
      xSpeed = driveController.getRawAxis(4);
      zRot = driveController.getRawAxis(0);
    }

    //Multiplies the motor speeds with sensitivity
    ySpeed = ySpeed * sens;
    xSpeed = xSpeed * sens;
    zRot = zRot * sens;

    // Adds deadzone and applies motor speed values to motors 
    if (Math.abs(ySpeed) > 0.2 || Math.abs(xSpeed) > 0.2 || Math.abs(zRot) > 0.2) {
      frontLeft.set(ySpeed + xSpeed + zRot);
      frontRight.set(ySpeed + -xSpeed + -zRot);
      rearLeft.set(ySpeed + -xSpeed + zRot);
      rearRight.set(ySpeed + xSpeed + -zRot);
    }

    // Engages the climbing (winch) motors when left bumper or right bumper button is pressed
    if (driveController.getRightBumper()) {
      climb2.set(0.6);
    } else if (driveController.getRightTriggerAxis() > 0.1) {
      climb2.set(-0.6);
    } else {
      climb2.stopMotor();
    }
    
    if (driveController.getLeftBumper()) {
      climb1.set(0.6);
    } else if (driveController.getLeftTriggerAxis() > 0.1) {
      climb1.set(-0.6);
    } else {
      climb1.stopMotor();
    }

    // Engages the shooter motors when either the right bumper or right trigger is pressed
    shooterSpeed = SmartDashboard.getNumber("Shooter Manual", shooterSpeed);
    if (shootController.getRightBumper()) {
      shooterLeft.set(shooterSpeed);
    } else if (shootController.getLeftBumper()) {
      shooterLeft.set(0.48);
    } else if (shootController.getLeftTriggerAxis() > 0.1) {
      shooterLeft.set(-0.2);
    } else if (shootController.getRightTriggerAxis() > 0.1) {
      if (SmartDashboard.getBoolean("Shooter Override", false)) {
        shooterLeft.set(shooterSpeed);
      } else {
        shooterLeft.set(shootController.getRightTriggerAxis());
      }
    } else {
      shooterLeft.stopMotor();
    }

    // Engages the intake motors either when the left stick is pushed forward (positive) or backward (negative)
    if (shootController.getLeftY() > 0.6) {
      intake.set(0.75);
    } else if (shootController.getLeftY() < -0.6) {
      intake.set(-0.75);
    } else {
      intake.stopMotor();
    }    

    // // Sets LED's colors based on shooter speed
    // if (shooterLeft.get() > 0.9) {
    //   led.set(-0.11);
    // } else if (shooterLeft.get() > 0.6) {
    //   led.set(0.61);
    // } else if (shooterLeft.get() > 0.3) {
    //   led.set(0.65);
    // } else if (shooterLeft.get() > 0.1) {
    //   led.set(0.71);
    // } else if (climb2.get() > 0.1 || climb1.get() > 0.1) {
    //   led.set(-0.41);
    // } else {
    //   led.set((invertedDrive) ? 0.61 : 0.77);
    // }

    // Engages the uptake motor when the right stick is pushed forward (postive) or backward (negative)
    if (shootController.getRightY() > 0.6) {
      uptake.set(0.55);
      index1.set(0.5);
    } else if (shootController.getRightY() < -0.6) {
      uptake.set(-0.55);
      index1.set(-0.5);
    } else {
      uptake.stopMotor();
      index1.stopMotor();
    }

    if (shootController.getAButton()) {
      limelightLED.setNumber(3);
      pipeline.setNumber(1);
      autoAlignTurret();
    } else if (shootController.getXButton()) {
      limelightLED.setNumber(3);
      pipeline.setNumber(1);
      autoDistance();
    } else {
      limelightLED.setNumber(1);
      // pipeline.setNumber(0);
    }

    update();

  }

  protected void update() {
    SmartDashboard.putNumber("Shooter", shooterLeft.get() * 100);
    SmartDashboard.putNumber("Shooter Manual", shooterSpeed);

    SmartDashboard.putNumber("Sensitivity", sens);
    SmartDashboard.putNumber("Timer", Timer.getMatchTime());

    // SmartDashboard.putData("Robot Drive", m_robotDrive);

    SmartDashboard.putString("Inverted Drive", (invertedDrive) ? "Inverted" : "Normal");
  }

  protected void drive(double frontL, double rearL, double frontR, double rearR) {
    frontLeft.set(frontL);
    frontRight.set(frontR);
    rearLeft.set(rearL);
    rearRight.set(rearR);
  }

  protected void stopDrive() {
    frontLeft.stopMotor();
    rearLeft.stopMotor();
    frontRight.stopMotor();
    rearRight.stopMotor();
  }

  protected void autoAlignTurret() {
    // Fine tuning constants
    double kpAim = 0.1;
    double minAimCommand = 0.1;
    
    double leftWheels = 0.0;
    double rightWheels = 0.0;
    double x = tx.getDouble(0.0);

    double headingError = x + 2;
    double steeringAdjust = 0.0;

    // If the target is too far right, then turn left
    // If the target is too far left, then turn right
    if (x > 0.5) {
      steeringAdjust = kpAim * headingError - minAimCommand;
    } else if (x < -0.5) {
      steeringAdjust = kpAim * headingError + minAimCommand;
    }

    leftWheels += steeringAdjust;
    rightWheels -= steeringAdjust; 

    drive(leftWheels, leftWheels, rightWheels, rightWheels);
  }

  private void autoDistance() {
    double y = ty.getDouble(0.0) - 6;
    
    // Adjust our distance to the target (y value)
    if (y > 0.5) {
      drive(0.25, 0.25, 0.25, 0.25);      
    } else if (y < -0.5) {
      drive(-0.25, -0.25, -0.25, -0.25);
    }
  }

}

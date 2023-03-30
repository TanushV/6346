// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.Math.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

//These 3 libraries are neccessary for the motors to run. These are can bus motors, not pwm.
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

//Camera imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.trajectory.*;







/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Set the device id to the same as what it was set to in the rev robotics client
  //Encoder encoder = new Encoder(0, 0);

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_armController = new XboxController(1);
  
  //used to be MotorController object
  CANSparkMax m_frontLeft = new CANSparkMax(4, MotorType.kBrushless);
  //RelativeEncoder encoder = m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  CANSparkMax m_rearLeft = new CANSparkMax(3, MotorType.kBrushless);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  CANSparkMax m_frontRight = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(1, MotorType.kBrushless);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right); //Drive control with two motors per side
  //DifferentialDrive m_drive = new DifferentialDrive(m_rearLeft, m_rearRight); //Drive control with one motor per side
  //DifferentialDrive m_drive = new DifferentialDrive(m_frontLeft, m_frontRight); //Drive control with one motor per side

  //INITIALIZATIONS FOR ARM
  //Note MotorControllerGroup will not allow for inverting the rightarm Motor
  //Using Leader-Follower commands
  CANSparkMax m_leftArm_Leader = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax m_rightArm_Follower = new CANSparkMax(6, MotorType.kBrushless);
  MotorControllerGroup m_arm = new MotorControllerGroup(m_leftArm_Leader, m_rightArm_Follower);

  //INITIALIZATIONS FOR THE ARM WINCH
  CANSparkMax m_armWinch = new CANSparkMax(7, MotorType.kBrushless);

  CANSparkMax m_pickup = new CANSparkMax(8, MotorType.kBrushless);
  //CANSparkMax m_unassigned = new CANSparkMax(9, MotorType.kBrushless);


  //Absolute Encoder declaration
  private DutyCycleEncoder m_Encoder = new DutyCycleEncoder(0);

  //Initializations for Limit Switches
  DigitalInput LS_front = new DigitalInput(2);
  DigitalInput LS_rear = new DigitalInput(1);
  private static final boolean TurboAllowed = true;

  //GYRO declaration
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  RelativeEncoder Winch_Encoder = m_armWinch.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  RelativeEncoder FrontLeftEncoder = m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  RelativeEncoder Pickup_Encoder = m_pickup.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  RelativeEncoder armRotate_Encoder = m_leftArm_Leader.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  double enc_wheelZero;
  double autoMove_offset;
  double enc_pickupZero;
  double autoPickup_offset;
  double enc_armRotateZero;
  double autoRotate_offset;
  double enc_WinchZero;
  double autoWinch_offset;

  boolean incomplete = true;
  boolean init = true;

  /* 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_rearRight.setInverted(true);

    m_rightArm_Follower.setInverted(true);
    m_Encoder.setDistancePerRotation(0.2083333333333333);

    SmartDashboard.setDefaultNumber("Autonomous Number #", 1);

    gyro.calibrate();

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double EncoderOutput = m_Encoder.get();
    SmartDashboard.putNumber("Encoder Output", EncoderOutput);
    SmartDashboard.putBoolean("Front Limit Switch", LS_front.get());
    SmartDashboard.putBoolean("Rear Limit Switch", LS_rear.get());
    SmartDashboard.putNumber("Winch Encoder", Winch_Encoder.getPosition());
    SmartDashboard.putNumber("winch zero", enc_WinchZero);
    SmartDashboard.putNumber("Auto Winch Offset", autoWinch_offset);
    SmartDashboard.putNumber("Arm Position", armRotate_Encoder.getPosition());


  }
//test
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    enc_wheelZero = FrontLeftEncoder.getPosition();
    enc_pickupZero = Pickup_Encoder.getPosition();
    enc_armRotateZero = armRotate_Encoder.getPosition();
    enc_WinchZero = Winch_Encoder.getPosition();
    incomplete = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     if (incomplete == true){
    switch (m_autoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
       
      //Arm lowers until it hits the Front Limit Switch then stops
        while(LS_front.get() == true){
          m_arm.set(0.25);
        }
          m_arm.set(0);
          enc_armRotateZero = armRotate_Encoder.getPosition();
        
        //Pickup spins and spits out cargo then stops
        autoPickup_offset = 20;
        while(Pickup_Encoder.getPosition() < (enc_pickupZero + autoPickup_offset)){
          m_pickup.set(0.5);
        }
          m_pickup.set(0);
      
        //Arm rotates to vertical position and then stops
        autoRotate_offset = 60;  
        while(armRotate_Encoder.getPosition() > (enc_armRotateZero - autoRotate_offset)) {
          m_arm.set(-.25);
          if(armRotate_Encoder.getPosition() < (enc_armRotateZero - autoRotate_offset)) {
            m_arm.set(0);
          }
        }
        
        //Wheels spin and the robot drives backwards to exit the community
        autoMove_offset = 60;
        while(FrontLeftEncoder.getPosition() < (enc_wheelZero + autoMove_offset)) {
          m_drive.arcadeDrive(0.50, 0);
          if(FrontLeftEncoder.getPosition() > (enc_wheelZero + autoMove_offset)) {
            for(int i = 0; i < 10; i++){
            m_drive.arcadeDrive(-0.50, 0);
            }
          }
        }
        
        m_arm.set(0);
        m_drive.arcadeDrive(0,0);
        incomplete = false;
        break;
      default:
        break;
      }
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_arm.set(0);
    m_drive.arcadeDrive(0,0);
    m_pickup.set(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if (incomplete == true){
      enc_WinchZero = Winch_Encoder.getPosition();
      init = false;
    }
    
  
    //Drive Code: Controller #0 Left Stick Y = Power & Right Stick X = Turn. Holding Right Bumper will increase the rotation speed
    //Status: Complete (2/23/2023) Turning set to 75% for better handling
    //DO NOT MODIFY UNTIL ALL OTHER CODE COMPLETE
    double slowDrive = 0.5; //slowDrive value <1 modifies the normal drive speed (0.5 = half speed)
    if (m_driverController.getRightBumper() == true & TurboAllowed)
     {
       //When Right Bumper is held, enable fine turning control
       m_drive.arcadeDrive(m_driverController.getLeftY(), -m_driverController.getRightX()*0.5, true);
     }
    else
     { 
       m_drive.arcadeDrive((m_driverController.getLeftY()*0.9), -(m_driverController.getRightX()*0.8), true);
     }  
    
    //Arm Rotate Code: Controller #1 Left Stick Y = Power. Holding Right Bumper will increase the rotation speed
    double slowArm = 0.5;
    //slowArm value <1 modifies the arm rotation speed (0.5 = half speed)
    if (Math.abs(m_armController.getLeftY()) > 0.05){ //Accounts for slight stick drift by increasing deadzone by 5%
        m_arm.set(-m_armController.getLeftY()*slowArm);
      }
      else{ 
      m_arm.set(0);
    }
  
   
    //Arm Extend Code: Right Stick Y = Power. Holding Right Bumper will increase the rotation speed
    //Status: Complete 2/25/2023
    //NOTE: All speed commands to the motors should use the m_leftArm_Leader
    
    if (incomplete == false){
    double slowWinch = 1; //slowArm value <1 modifies the arm rotation speed (0.5 = half speed)
    autoWinch_offset = -500; // Full Winch extension + extra
    if (Math.abs(m_armController.getRightY()) > 0.05){ //Accounts for slight stick drift by increasing deadzone by 5%
      
      if (m_armController.getRightY()< 0 && Winch_Encoder.getPosition() < (enc_WinchZero + autoWinch_offset)){
        m_armWinch.set(0);//stop winch
      }
      else if (m_armController.getRightY()> 0 && Winch_Encoder.getPosition() > (enc_WinchZero)){ //Winch Fully Retracted
        m_armWinch.set(0);//stop winch
      }
      else { 
        m_armWinch.set((m_armController.getRightY()*slowWinch));
      }
    }  
    else { m_armWinch.set(0);}
  }
  else{
    double slowWinch = 1; //slowArm value <1 modifies the arm rotation speed (0.5 = half speed)
    if (Math.abs(m_armController.getRightY()) > 0.05){
      m_armWinch.set((m_armController.getRightY()*slowWinch));
    }
    else { m_armWinch.set(0);}
  }
    //Limit Switch Code: When limit switch is engaged, check Left Stick input and stop motor continuing into the switch
    //Status: Complete 2/25/2023
    //LIMIT SWITCH CODE MUST BE LAST
   if (LS_rear.get() == false){ //if Normally Open contact is closed
      if (m_armController.getLeftY()> 0){//if arm is still recieving a command to move into limit switch (+y is towards front)
        m_arm.set(0);//stop arm
      }
    }
    
    if (LS_front.get() == false){ //if Normally Open contact is closed
        if (m_armController.getLeftY()< 0){ //if arm is still recieving a command to move into limit switch (-y is towards rear)
        m_arm.set(0); //stop arm
      }
    }

    //Pickup code
    //Left and Right Trigger control the roller direction
    if((m_armController.getRightTriggerAxis() > 0.05) && (m_armController.getLeftTriggerAxis() < 0.05)){ //suck in cube
      m_pickup.set(m_armController.getRightTriggerAxis() * -0.6);
    }
    else if((m_armController.getLeftTriggerAxis() > 0.05) && (m_armController.getRightTriggerAxis() < 0.05)){ //spit out cube
      m_pickup.set(m_armController.getLeftTriggerAxis() * +0.6);
    }
    else {
      m_pickup.set(0);
    }

     
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
     
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
     
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
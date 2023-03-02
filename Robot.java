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
  
  //INITIALIZATIONS FOR THE ARM WINCH
  CANSparkMax m_armWinch = new CANSparkMax(7, MotorType.kBrushless);

  CANSparkMax m_handWrist = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax m_handgripper = new CANSparkMax(9, MotorType.kBrushless);


  //Absolute Encoder declaration
  private DutyCycleEncoder m_Encoder = new DutyCycleEncoder(0);

  //Initializations for Limit Switches
  DigitalInput LS_front = new DigitalInput(1);
  DigitalInput LS_rear = new DigitalInput(2);
  private static final boolean TurboAllowed = false;

  //ADIS16470_IMU gyro = new ADIS16470_IMU();
  double slowArm = 0.25;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_rightArm_Follower.follow(m_leftArm_Leader, true);//The right arm motor matches voltage and inverts direction of the left arm
    //NOTE: All speed commands to the motors should use the m_leftArm_Leader
    m_Encoder.setDistancePerRotation(0.2083333333333333);

    //encoder.setDistancePerPulse(1./4096.);
    //gyro.calibrate();   
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
    SmartDashboard.putNumber("Drive X Axis", m_driverController.getRightX());
    SmartDashboard.putNumber("Drive Y Axis", m_driverController.getRightY());
    




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
     
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
        
        break;
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
     
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //Drive Code: Controller #0 Left Stick Y = Power & Right Stick X = Turn. Holding Right Bumper will increase the rotation speed
    //Status: Complete (2/23/2023) Turning set to 75% for better handling
    //DO NOT MODIFY UNTIL ALL OTHER CODE COMPLETE
    double slowDrive = 0.5; //slowDrive value <1 modifies the normal drive speed (0.5 = half speed)
    if (m_driverController.getRightBumper() == true & TurboAllowed)
    {
      //When Right Bumper is held, enable turbo mode
      m_drive.arcadeDrive(-m_driverController.getRightY(), -m_driverController.getRightX()*0.5);
    }
   else
    { 
      m_drive.arcadeDrive(-(m_driverController.getRightY()*slowDrive), -(m_driverController.getRightX()*0.75*slowDrive));
      
    }  

    //Arm Rotate Code: Controller #1 Left Stick Y = Power. Holding Right Bumper will increase the rotation speed
    //Status: In progress: Basic functionality (2/22/2023)
    //NOTE: All speed commands to the motors should use the m_leftArm_Leader
     //slowArm value <1 modifies the arm rotation speed (0.25 = 1/4 speed)
    
    //Encoder values should be close to 0 when vertical, (+) to the rear, and (-) to the front
    //NOTE: When the chain comes off these numbers must be found again
    //To zero encoder, put arm into vertical position and cycle power    
    double enc_Vertical = 0.230;
    double enc_RearLowest = 1.606;
    double enc_FrontLowest = -1.205;
    double enc_position = m_Encoder.get();
    boolean DangerZone;

    double slowArm = 0.5; //slowArm value <1 modifies the arm rotation speed (0.5 = half speed)
// If the arm is moving forward and is within 0.1 rotatations of the lowest it can go in the rear, speed
    // will be set to 10% of maximum speed. Will not apply if arm is moving backwards. Does not need to apply
    // a new speed and can be completely separate from the code for moving the arm if i'm not mistaken
    if(m_armController.getLeftY() > 0 && enc_position > (enc_RearLowest - 0.1)) {
      slowArm = 0.1;
      DangerZone = true;
    }

    // If the arm is moving backwards and is within 0.1 rotatations of the lowest it can go in the front, speed
    // will be set to 10% of maximum speed. Will not apply if arm is moving forwards. Does not need to apply
    // a new speed and can be completely separate from the code for moving the arm if i'm not mistaken
    else if(m_armController.getLeftY() < 0 && enc_position < (enc_FrontLowest + 0.1)) {
      slowArm = 0.1;
      DangerZone = true;
    }
    else {
      DangerZone = false;
    }


    /* enc_position - enc_RearLowest < .05 && -1.0*enc_position - -1.0*enc_FrontLowest < .05 */
    if (m_armController.getRightBumper() == false && DangerZone == false)
    {
      slowArm = 0.5;
    }

    m_leftArm_Leader.set((m_armController.getLeftY())*slowArm);

   /* else if(enc_position - enc_RearLowest < .05 && -1.0*enc_position - -1.0*enc_FrontLowest < .05)
    {
      slowArm = .25;
      m_leftArm_Leader.set((m_armController.getLeftY()*slowArm));
    }else{ // IN THE DANGER ZONE
      slowArm = .128; // might need to be larger
      m_leftArm_Leader.set((m_armController.getLeftY()*slowArm));
    } */ 
    
    //Arm Extend Code: Right Stick Y = Power. Holding Right Bumper will increase the rotation speed
    //Status: Complete 2/25/2023
    //NOTE: All speed commands to the motors should use the m_leftArm_Leader
    double slowWinch = 0.5; //slowArm value <1 modifies the arm rotation speed (0.5 = half speed)
    if (m_armController.getRightBumper() == true && TurboAllowed)
    {
      m_armWinch.set((m_armController.getRightY()));
    }
   else
    { 
      m_armWinch.set((m_armController.getRightY()*slowWinch));
    }  


    //Limit Switch Code: When limit switch is engaged, check Left Stick input and stop motor continuing into the switch
    //Status: Complete 2/25/2023
    //LIMIT SWITCH CODE MUST BE LAST
   if (LS_front.get() == false){ //if Normally Open contact is closed
      if (-m_armController.getLeftY()> 0){//if arm is still recieving a command to move into limit switch (+y is towards front)
        m_leftArm_Leader.set(0);//stop arm
      }
    }
    
    if (LS_rear.get() == false){ //if Normally Open contact is closed
        if (-m_armController.getLeftY()< 0){ //if arm is still recieving a command to move into limit switch (-y is towards rear)
        m_leftArm_Leader.set(0); //stop arm
      }
    }


    //Encoder Code:
    //Status: In Progress: Goal to zero encoder when arm is vertical with chain
    //Note encoder loses # of rotations after power cycle (e.g. position 13.5 --> 0.5)
    /* if(m_armController.getAButton() == true) { //DO NOT HOLD BUTTON. Just tap for numbers or you will massivelly delay terminal outputs
      System.out.printf("%.3f %n", m_Encoder.get()); //Prints Encoder output to 3 decimal places
    } */
    //Turns the wrist left
    if(m_armController.getXButton()) {
      m_handWrist.set(-0.1);
    }
    //Turns the wrist right
    if(m_armController.getBButton()) {
      m_handWrist.set(0.1);
    }

    if((m_armController.getRightTriggerAxis() > 0.05) && (m_armController.getLeftTriggerAxis() < 0.05))
    {
      m_handgripper.set(m_armController.getRightTriggerAxis() * 0.25);
    }

    if(m_armController.getLeftTriggerAxis() > 0.05)
    {
      m_handgripper.set(m_armController.getLeftTriggerAxis() * -0.25);
    }
    
    
    


    

   //SmartDashboard.putBoolean("DangerZone", DangerZone);
   





  
     
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

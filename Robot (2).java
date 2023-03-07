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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

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

  
  private final XboxController m_controller = new XboxController(0);
  
  //used to be MotorController object
  CANSparkMax m_frontLeft = new CANSparkMax(4, MotorType.kBrushless);
  //RelativeEncoder encoder = m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  CANSparkMax m_rearLeft = new CANSparkMax(1, MotorType.kBrushless);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(2, MotorType.kBrushless);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

  //DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right); //Drive control with two motors per side
  DifferentialDrive m_drive = new DifferentialDrive(m_rearLeft, m_rearRight); //Drive control with one motor per side

  //ADIS16470_IMU gyro = new ADIS16470_IMU();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_left.setInverted(true); // this is for arcade drive

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
    //encoder.get();


    m_drive.arcadeDrive(-m_controller.getLeftY(), m_controller.getRightX());

    // Runs the rear left motor at 50% speed
    if(m_controller.getXButton()) {
      m_rearLeft.set(0.5);
    } 
    else {
        m_rearRight.set(0);
    }

    // Runs the front right motor at 50% speed
    if(m_controller.getYButton()) {
      m_frontRight.set(0.5);
    } else {
      m_frontRight.set(0);
    }
    
    if(m_controller.getLeftBumper()) {
        double TheYaw = gyro.getAngle();
        Math.sin(TheYaw)*555.66;
        if(TheYaw > 2 && <= 5) {

        }
        else if(TheYaw > 5 && <= 8){

        }
        else if(TheYaw > 8 && <= 11){

        }
        else if(TheYaw > 11 && <= 14){

        }
        else if(TheYaw > 14){

        }
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

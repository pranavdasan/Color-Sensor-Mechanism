// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CANSparkMax motor;  
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  private Color colorDetected;
  private double redValue;
  private double blueValue;
  private double greenValue;

  private static Timer timer;

  // Time to delay the motor (in seconds)
  private final int motorDelay = 5;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

    m_robotContainer = new RobotContainer();

    timer = new Timer(); 
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
    colorDetected = colorSensor.getColor();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    redValue = colorDetected.red;
    blueValue = colorDetected.blue;
    greenValue = colorDetected.green;

    
    // Color detected by the sensor is mainly red
    if (redValue > blueValue && redValue > greenValue) {      
      doubleSolenoid.set(DoubleSolenoid.Value.kForward); 

      motor.set(1);

      timer.delay(motorDelay);
      
      motor.stopMotor();

    } else if (blueValue > redValue && blueValue > greenValue) { // Color dected by sensor is mainly blue
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

      motor.stopMotor();

    } else { // Color that is not mainly red or blue detected
      doubleSolenoid.set(DoubleSolenoid.Value.kOff);
      
      motor.stopMotor();
    }   
  }
}

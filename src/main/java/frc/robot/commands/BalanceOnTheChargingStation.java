// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class BalanceOnTheChargingStation extends CommandBase {

  private static final double DRIVE_SPEED = 0.4;
  private static final double REDUCED_DRIVE_SPEED = 0.2;
  private static final double PITCH_TOLERANCE = 2.0;
  private static final double REDUCED_SPEED_PITCH = 8.0;
  private static final double TIME_LIMIT = 1.0;
  private Drivetrain m_drivetrain;
  private Timer m_timer;

  /** Creates a new BalanceOnTheChargingStation. */
  public BalanceOnTheChargingStation(Drivetrain subsystem) {
    m_drivetrain = subsystem;
    addRequirements(m_drivetrain);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("pitch is: " + m_drivetrain.getPitchDeg());
    if (m_drivetrain.getPitchDeg() > PITCH_TOLERANCE){
      if (m_drivetrain.getPitchDeg() < REDUCED_SPEED_PITCH){
        m_drivetrain.drive(new Translation2d(REDUCED_DRIVE_SPEED, 0), 0, true);
        m_timer.reset();
      } 
        else {m_drivetrain.drive(new Translation2d(DRIVE_SPEED, 0), 0, true);
        m_timer.reset();
      }
      
      
    }
        else if (m_drivetrain.getPitchDeg() < -PITCH_TOLERANCE){
          if (m_drivetrain.getPitchDeg() < -REDUCED_SPEED_PITCH){
            m_drivetrain.drive(new Translation2d(-REDUCED_DRIVE_SPEED, 0), 0, true);
            m_timer.reset();
          } else {
          m_drivetrain.drive(new Translation2d(-0.2, 0), 0, true);
           m_timer.reset();

          }
         
          
          
          
      
        }
    else {
      m_drivetrain.stop();
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > TIME_LIMIT;
  }
}

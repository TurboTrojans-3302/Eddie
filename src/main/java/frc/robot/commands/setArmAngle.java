// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;




public class setArmAngle extends CommandBase {

  private static final double TOLERANCE = 2.5;
  /** Creates a new setArmAngle. */
  Arm m_arm;
  double armSpeed;
  double targetArmAngle;
  boolean finished;
  double currentAngle;


  public setArmAngle(double angle, double speed){
    m_arm = RobotContainer.getInstance().m_arm;
    addRequirements(m_arm);
    targetArmAngle = angle;
    armSpeed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_arm.getElbowAngle();  

    if (targetArmAngle < currentAngle){
      m_arm.elbowMove(-armSpeed); 
    } else if (targetArmAngle > currentAngle) {
      m_arm.elbowMove(armSpeed); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.elbowMove(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = m_arm.getElbowAngle();
    return targetArmAngle > currentAngle - TOLERANCE && targetArmAngle < currentAngle + TOLERANCE;
  }
}


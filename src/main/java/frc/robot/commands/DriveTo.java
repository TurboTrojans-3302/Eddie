// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTo extends CommandBase {
  private static final double DESTINATION_TOLERANCE = 0.050;
  /** Creates a new DriveTo. */

  private Pose2d m_dest;
  private Drivetrain m_drivetrain;
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(Drivetrain.MAX_SPEED, Drivetrain.MAX_SPEED/2.0);
  private double m_distance;

  public DriveTo(Pose2d destination) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);

    m_dest = destination;
  }

  public DriveTo(double x, double y){
    this(new Pose2d(new Translation2d(x, y), new Rotation2d()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d delta = m_dest.minus(m_drivetrain.getPose2d()).getTranslation();
    m_distance = delta.getNorm();
    Rotation2d direction = delta.getAngle();

    double current_speed = m_drivetrain.getVelocityVector().getNorm();

    final TrapezoidProfile profile = new TrapezoidProfile(m_constraints,
                                                          new State(m_distance, 0),
                                                          new State(0, current_speed));

    double speed = profile.calculate(0.026).velocity;
    m_drivetrain.drive(new Translation2d(speed, direction), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  public Translation2d getDelta() {
    return m_dest.minus(m_drivetrain.getPose2d()).getTranslation();
  }

  // public double getDistance() {
  //   return getDelta().getNorm();
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distance <  DESTINATION_TOLERANCE;
  }
}

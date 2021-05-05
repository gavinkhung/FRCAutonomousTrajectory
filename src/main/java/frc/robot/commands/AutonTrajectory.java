/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonTrajectory extends SequentialCommandGroup {

  /**
   * Rendezvous Auton Routine
   */
  public AutonTrajectory(Drivetrain drivetrain) {
    addCommands(
      getTrajectory(drivetrain)
    );
  }

  public Command getTrajectory(Drivetrain drivetrain){
    drivetrain.invertPathDirection(true);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(0, 0, new Rotation2d(3.048, 0)),
      new Pose2d(5, -1, new Rotation2d(0, -3.048))
    ), drivetrain.getTrajectoryConfig());

    
    RamseteCommand command = new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(2.0, 7.0),
      drivetrain.getFeedForward(),
      drivetrain.getKinematics(),
      drivetrain::getSpeeds,
      drivetrain.getLeftPIDController(),
      drivetrain.getRightPIDController(),
      drivetrain::setOutput,
      drivetrain
    );

    return command;
  }

}

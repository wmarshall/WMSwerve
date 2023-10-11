// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDriveTrain driveTrain = new SwerveDriveTrain();

  private Joystick joy = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    driveTrain.setDefaultCommand(
        driveTrain.DriveRelative(() -> {
          var fwd = MathUtil.applyDeadband(-joy.getY(), 0.1);
          fwd = fwd * fwd * fwd;
          var str = MathUtil.applyDeadband(joy.getX(), 0.1);
          str = str * str * str;
          return new ChassisSpeeds(
              SwerveDriveTrain.DRIVE_MAX_VELOCITY_METERS_PER_SECOND * fwd,
              SwerveDriveTrain.DRIVE_MAX_VELOCITY_METERS_PER_SECOND * str,
              0);
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

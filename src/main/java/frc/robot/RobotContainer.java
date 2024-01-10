// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // life may get better if we move this to a global/measure it
  private final double LOOP_DT = 0.02;

  private SwerveDriveTrain driveTrain = new SwerveDriveTrain();

  // sensors are not subsystems. Subsystems can _have_ sensors
  private ADIS16448_IMU imu = new ADIS16448_IMU();

  private Rotation2d desiredHeading;
  // 1 radian per second output per radian of error
  private PIDController rotationController = new PIDController(1, 0, 0);

  private Joystick joy = new Joystick(0);

  public RobotContainer() {
    desiredHeading = Rotation2d.fromDegrees(imu.getAngle());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    configureBindings();
  }

  private void configureBindings() {

    driveTrain.setDefaultCommand(
        driveTrain.DriveRelative(() -> {
          var fwd = MathUtil.applyDeadband(-joy.getY(), 0.1);
          fwd = fwd * fwd * fwd;
          var str = MathUtil.applyDeadband(joy.getX(), 0.1);
          str = str * str * str;

          var rotRatePct = MathUtil.applyDeadband(joy.getTwist(), 0.1);
          rotRatePct = rotRatePct * rotRatePct * rotRatePct;

          var maxHeadingChange = SwerveDriveTrain.DRIVE_MAX_OMEGA_RADIANS_PER_SECOND / LOOP_DT;

          desiredHeading = desiredHeading.plus(Rotation2d.fromRadians(maxHeadingChange * rotRatePct));

        return ChassisSpeeds.fromFieldRelativeSpeeds(
              SwerveDriveTrain.DRIVE_MAX_VELOCITY_METERS_PER_SECOND * fwd,
              SwerveDriveTrain.DRIVE_MAX_VELOCITY_METERS_PER_SECOND * str,
              rotationController.calculate(imu.getAngle(), desiredHeading.getRadians()),
              Rotation2d.fromDegrees(imu.getAngle()));
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

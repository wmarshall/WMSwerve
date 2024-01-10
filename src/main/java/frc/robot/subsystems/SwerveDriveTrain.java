package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.robot.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveDriveTrain implements Subsystem {

    public static final double WHEEL_BASE_INCHES = 18;
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(WHEEL_BASE_INCHES);
    public static final double TRACK_WIDTH_INCHES = 24;
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);

    // 0, 1, 2, 3 following quadrant numbering (CCW, starting NE)
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACK_WIDTH_METERS / 2, WHEEL_BASE_METERS / 2),
            new Translation2d(-TRACK_WIDTH_METERS / 2, WHEEL_BASE_METERS / 2),
            new Translation2d(-TRACK_WIDTH_METERS / 2, -WHEEL_BASE_METERS / 2),
            new Translation2d(TRACK_WIDTH_METERS / 2, -WHEEL_BASE_METERS / 2));

    // From https://www.revrobotics.com/rev-21-3005/
    // Reductions are always measured as turns-in/turns-out
    public static final double STEERING_MOTOR_REDUCTION = 9424.0 / 203.0;
    public static final double DRIVE_MOTOR_PINION_TEETH = 14;
    // Spot check - with a 12 tooth pinion this is ~5.5:1, as stated by the docs
    public static final double DRIVE_MOTOR_REDUCTION = (22.0 / DRIVE_MOTOR_PINION_TEETH) * (45.0 / 15.0);
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 3;
    public static final double DRIVE_WHEEL_CIRCUMFRENCE_INCHES = DRIVE_WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double DRIVE_WHEEL_CIRCUMFRENCE_METERS = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFRENCE_INCHES);

    // Spec is 15.76 ft/s on a 14t pinion, derated slightly
    public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = Units.feetToMeters(15);

    // Just a guess
    public static final double DRIVE_MAX_OMEGA_RADIANS_PER_SECOND = Math.PI;

    // Just a guess since I can't find supporting documentation - we should only
    // ever be commanding 90 degree rotation and we should be able to do that in
    // ~200ms
    public static final double MODULE_MAX_SLEW_PER_SECOND = (Math.PI / 2) / 0.2;
    public static final double SERVICE_DT = 0.02;

    private static final class Module {
        private final CANSparkMax drive, steer;
        private SwerveModuleState desired;

        public Module(int driveID, int turnID, Rotation2d zeroOffset) {
            drive = driveFactory(driveID, false);
            steer = steerFactory(
                    turnID,
                    zeroOffset, true, false, false);
            desired = getState();
        }

        private static CANSparkMax driveFactory(int deviceID, boolean motorInverted) {
            var drive = new CANSparkMax(deviceID, MotorType.kBrushless);
            Util.handleREVLibErr(drive.restoreFactoryDefaults());
            Util.handleREVLibErr(drive.setIdleMode(IdleMode.kBrake));
            Util.handleREVLibErr(drive.enableVoltageCompensation(12));

            // WPILib setInverted doesn't allow returning an error :'(
            drive.setInverted(motorInverted);
            Util.handleREVLibErr(drive.getLastError());

            // native units are Rotations, RPM
            // we want meters/ meters per second
            var encoder = drive.getEncoder();
            Util.handleREVLibErr(
                    encoder.setPositionConversionFactor(DRIVE_WHEEL_CIRCUMFRENCE_METERS / DRIVE_MOTOR_REDUCTION));
            Util.handleREVLibErr(encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60));

            var controller = drive.getPIDController();
            Util.handleREVLibErr(controller.setFeedbackDevice(encoder));
            // In units of output fraction [-1, 1] per m/s of setpoint
            Util.handleREVLibErr(controller.setFF(.2));
            // In units of output fraction [-1, 1] per m/s of error
            Util.handleREVLibErr(controller.setP(.01));

            return drive;
        }

        private static CANSparkMax steerFactory(int deviceID, Rotation2d zeroOffset, boolean headingEncoderInverted,
                boolean relativeEncoderInverted, boolean motorInverted) {
            var steer = new CANSparkMax(deviceID, MotorType.kBrushless);
            Util.handleREVLibErr(steer.restoreFactoryDefaults());
            Util.handleREVLibErr(steer.setIdleMode(IdleMode.kBrake));
            Util.handleREVLibErr(steer.enableVoltageCompensation(12));

            // WPILib setInverted doesn't allow returning an error :'(
            steer.setInverted(motorInverted);
            Util.handleREVLibErr(steer.getLastError());

            // Assume NEO550, don't smoke the poor motor
            Util.handleREVLibErr(steer.setSmartCurrentLimit(20));

            // The duty cycle encoder's native units are [0, 1) rotations
            // We want to scale to radians, then apply the known radian offset,
            // then seed the relative encoder.
            var heading_encoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
            // Rotations -> radians, RPM -> radians per second
            Util.handleREVLibErr(heading_encoder.setPositionConversionFactor(2 * Math.PI));
            Util.handleREVLibErr(
                    heading_encoder.setVelocityConversionFactor(heading_encoder.getPositionConversionFactor() / 60));
            Util.handleREVLibErr(heading_encoder.setInverted(headingEncoderInverted));

            // Each module has its own zero offset:
            // if we're passed a zero offset of -pi/2, wrap that to 3pi/2.
            // if passed an offset of 5pi/2, wrap to pi/2
            var wrappedZeroOffset = MathUtil.angleModulus(zeroOffset.getRadians());
            Util.handleREVLibErr(heading_encoder.setZeroOffset(wrappedZeroOffset));

            // The duty cycle encoder now reports the heading of the module in
            // range [0, 2pi)
            // angleModulus allows us to convert that to [-pi, pi), with 0 still
            // as "north"
            var true_heading = MathUtil.angleModulus(heading_encoder.getPosition());

            // We use the absolute encoder for seeding but the relative encoder for control:
            // - this allows us to survive the absolute encoder becoming unplugged during
            // the match,
            // - as well as have higher control resolution (1024 CPR on the absolute
            // encoder, 42*STEERING_MOTOR_REDUCTION ~=1949 CPR for the relative)
            // - the relative encoder is able to report negative readings w/o us needing to
            // do any trickery here
            // - This approach may introduce backlash concerns, but we'll leave that for
            // experimental determination
            var motor_encoder = steer.getEncoder();
            Util.handleREVLibErr(motor_encoder.setInverted(relativeEncoderInverted));
            // Rotations -> radians, RPM -> radians per second
            Util.handleREVLibErr(motor_encoder.setPositionConversionFactor(2 * Math.PI / STEERING_MOTOR_REDUCTION));
            Util.handleREVLibErr(
                    motor_encoder.setVelocityConversionFactor(motor_encoder.getPositionConversionFactor() / 60));

            // Seed the relative encoder with the true heading
            Util.handleREVLibErr(motor_encoder.setPosition(true_heading));

            // Configire PID
            var controller = steer.getPIDController();
            Util.handleREVLibErr(controller.setFeedbackDevice(motor_encoder));
            Util.handleREVLibErr(controller.setPositionPIDWrappingEnabled(true));
            Util.handleREVLibErr(controller.setPositionPIDWrappingMinInput(-Math.PI));
            Util.handleREVLibErr(controller.setPositionPIDWrappingMaxInput(Math.PI));

            // In units of output fraction [-1, 1] per radian of error
            Util.handleREVLibErr(controller.setP(1));
            // In units of output fraction [-1, 1] per radian of error per millisecond
            Util.handleREVLibErr(controller.setD(.001));

            // Enable the controller
            Util.handleREVLibErr(controller.setReference(motor_encoder.getPosition(), ControlType.kPosition));

            return steer;
        }

        public Rotation2d getRotation2d() {
            return new Rotation2d(steer.getEncoder().getPosition());
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    drive.getEncoder().getVelocity(),
                    getRotation2d());
        }

        public void setDesiredState(SwerveModuleState desired) {
            this.desired = desired;
        }

        /**
         * Like setDesiredState but doesn't rotate if the desired velocity is 0
         *
         * @param desired
         */
        public void setDesiredStatePersistAngle(SwerveModuleState desired) {
            if (desired.speedMetersPerSecond == 0) {
                desired.angle = getRotation2d();
            }
            setDesiredState(desired);
        }

        public SwerveModuleState getDesiredState() {
            return this.desired;
        }

        public void service() {
            var currentRotation = getRotation2d();
            // Assumption here - all "equivalent" states are actually equivalent
            // for our purposes. Lift this optimize call if that's not true
            var optimized = SwerveModuleState.optimize(desired, currentRotation);
            var rotationDelta = optimized.angle.minus(currentRotation);

            // Do a little baby motion profile by figuring out how long it
            // should take us to do this rotation and moving one timestep along
            // that line. When DT > timeToComplete, we'll go all the way to the end
            // e.x. rotation change of pi/2 can happen in 200ms
            var timeToCompleteSeconds = rotationDelta.getRadians() / MODULE_MAX_SLEW_PER_SECOND;
            var limitedSlewRotation = currentRotation.interpolate(
                    optimized.angle,
                    // TODO - it might be wise to not assume a constant timestep
                    MathUtil.clamp(SERVICE_DT / timeToCompleteSeconds, 0, 1));

            Util.handleREVLibErr(
                    drive.getPIDController().setReference(desired.speedMetersPerSecond, ControlType.kVelocity));
            Util.handleREVLibErr(
                    steer.getPIDController().setReference(limitedSlewRotation.getRadians(), ControlType.kPosition));
        }
    }

    private final Module module1, module2, module3, module4;

    public SwerveDriveTrain() {
        /*
         * The zeroing jig provided with MaxSwerve orients each module to:
         *
         * |------------|
         * |----....^...|
         * .....\...|...|
         * ......\..|...|
         * .......\.....|
         * ........\----|
         *
         * Where the arrow indicates the wheel direction.
         * When applied to all 4 modules though, this gives:
         * - Module 1 facing north
         * - Module 2 facing west
         * - Module 3 facing south
         * - Module 4 facing east
         *
         * Our constants for each module are captured from the use of the jig,
         * then we apply offsets to get them to face north
         */

        module1 = new Module(31, 32,
                Rotation2d.fromRotations(0.932).plus(new Rotation2d(0)));
        module2 = new Module(11, 12,
                Rotation2d.fromRotations(0.359).plus(new Rotation2d(Math.PI / 2)));
        module3 = new Module(21, 22,
                Rotation2d.fromRotations(0.517).plus(new Rotation2d(Math.PI)));
        module4 = new Module(41, 42,
                Rotation2d.fromRotations(0.575).plus(new Rotation2d(-Math.PI / 2)));

    }

    public void periodic() {
        module1.service();
        module2.service();
        module3.service();
        module4.service();
    }

    public Command X() {
        return new RunCommand(() -> {
            module1.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
            module2.setDesiredState(new SwerveModuleState(0, new Rotation2d(+Math.PI / 4)));
            module3.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
            module4.setDesiredState(new SwerveModuleState(0, new Rotation2d(+Math.PI / 4)));
        }, this);
    }

    public Command Zero() {
        return new RunCommand(() -> {
            module1.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
            module2.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
            module3.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
            module4.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
        }, this);
    }

    public Command DriveRelative(Supplier<ChassisSpeeds> s) {
        return new RunCommand(() -> {
            var states = kinematics.toSwerveModuleStates(s.get());
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DRIVE_MAX_VELOCITY_METERS_PER_SECOND);
            // TODO: 2nd order kinematics to prevent drift while rotating
            module1.setDesiredStatePersistAngle(states[0]);
            module2.setDesiredStatePersistAngle(states[1]);
            module3.setDesiredStatePersistAngle(states[2]);
            module4.setDesiredStatePersistAngle(states[3]);
        }, this);
    }
}
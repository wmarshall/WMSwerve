package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveDriveTrain implements Subsystem {

    public static double WHEEL_BASE_INCHES = 18;
    public static double TRACK_WIDTH_INCHES = 24;

    // From https://www.revrobotics.com/rev-21-3005/
    // Reductions are always measured as turns-in/turns-out
    public static double STEERING_MOTOR_REDUCTION = 9424.0 / 203.0;
    public static double DRIVE_MOTOR_PINION_TEETH = 12;
    // Spot check - with a 12 tooth pinion this is ~5.5:1, as stated by the docs
    public static double DRIVE_MOTOR_REDUCTION = (22.0 / DRIVE_MOTOR_PINION_TEETH) * (45.0 / 15.0);
    public static double DRIVE_WHEEL_DIAMETER_INCHES = 3;
    public static double DRIVE_WHEEL_CIRCUMFRENCE_INCHES = DRIVE_WHEEL_DIAMETER_INCHES * Math.PI;

    public static double L = WHEEL_BASE_INCHES / 2;
    public static double W = TRACK_WIDTH_INCHES / 2;
    public static double R = Math.sqrt(L * L + W * W);

    private final CANSparkMax FLDrive, FLSteer;
    private final CANSparkMax FRDrive, FRSteer;
    private final CANSparkMax RLDrive, RLSteer;
    private final CANSparkMax RRDrive, RRSteer;

    private double fwd, str, rcw;

    private CANSparkMax driveFactory(int deviceID, boolean motorInverted) {
        var drive = new CANSparkMax(deviceID, MotorType.kBrushless);
        Util.handleREVLibErr(drive.restoreFactoryDefaults());
        drive.setInverted(motorInverted);

        // native units are Rotations, RPM
        // we want Inches/ inches per second
        var encoder = drive.getEncoder();
        Util.handleREVLibErr(
                encoder.setPositionConversionFactor(DRIVE_WHEEL_CIRCUMFRENCE_INCHES / DRIVE_MOTOR_REDUCTION));
        Util.handleREVLibErr(encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60));

        return drive;
    }

    private CANSparkMax steerFactory(int deviceID, double zeroOffset, boolean headingEncoderInverted,
            boolean relativeEncoderInverted, boolean motorInverted) {
        var steer = new CANSparkMax(deviceID, MotorType.kBrushless);
        Util.handleREVLibErr(steer.restoreFactoryDefaults());

        steer.setInverted(motorInverted);
        // Assume NEO550, don't smoke the poor motor
        Util.handleREVLibErr(steer.setSmartCurrentLimit(30));

        // The duty cycle encoder's native units are [0, 1) rotations
        // We want to scale to degrees, then apply the known degree offset, then seed
        // the relative encoder.
        var heading_encoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
        // Rotations -> degrees, RPM -> degrees per second
        Util.handleREVLibErr(heading_encoder.setPositionConversionFactor(1 / 360.0));
        Util.handleREVLibErr(
                heading_encoder.setVelocityConversionFactor(heading_encoder.getPositionConversionFactor() / 60));
        Util.handleREVLibErr(heading_encoder.setInverted(headingEncoderInverted));

        // Each module has its own zero offset:
        // if we're passed a zero offset of -90, wrap that to 270.
        // if passed an offset of 450, wrap to +90
        var wrappedZeroOffset = MathUtil.inputModulus(zeroOffset, 0, 360);
        Util.handleREVLibErr(heading_encoder.setZeroOffset(wrappedZeroOffset));

        // The duty cycle encoder now reports the heading of the module in range [0,
        // 360)

        // Configure PID
        var controller = steer.getPIDController();
        Util.handleREVLibErr(controller.setFeedbackDevice(heading_encoder));
        Util.handleREVLibErr(controller.setPositionPIDWrappingEnabled(true));
        Util.handleREVLibErr(controller.setPositionPIDWrappingMinInput(0));
        Util.handleREVLibErr(controller.setPositionPIDWrappingMaxInput(360));
        // In units of output fraction [-1, 1] per degree of error
        Util.handleREVLibErr(controller.setP(0.05));
        // Enable the controller
        Util.handleREVLibErr(controller.setReference(heading_encoder.getPosition(), ControlType.kPosition));

        return steer;
    }

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
         * - FL facing west
         * - FR facing north
         * - RL facing south
         * - RR facing east
         * 
         * Our constants for each module are captured from the use of the jig,
         * then we apply offsets to get them to face north
         */
        // TODO: capture from REV Client, move to constants class

        double FLZeroOffsetDegrees = 0;
        double FRZeroOffsetDegrees = 0;
        double RLZeroOffsetDegrees = 0;
        double RRZeroOffsetDegrees = 0;

        FLDrive = driveFactory(0, false);
        FLSteer = steerFactory(
                1,
                FLZeroOffsetDegrees + 90,
                false,
                false,
                false);
        FRDrive = driveFactory(2,
                false);
        FRSteer = steerFactory(
                3,
                FRZeroOffsetDegrees + 0,
                false,
                false,
                false);
        RLDrive = driveFactory(4,
                false);
        RLSteer = steerFactory(
                5,
                RLZeroOffsetDegrees + 180,
                false,
                false,
                false);
        RRDrive = driveFactory(6,
                false);
        RRSteer = steerFactory(
                7,
                RRZeroOffsetDegrees + 270,
                false,
                false,
                false);
    }

    /*
     * Change an angle from [0, 360) to [-180, 180), wrapping input angles above 180 around to negatives
     */
    private double sparkMaxAngleToNormal(double sparkMaxAngle) {
        return MathUtil.inputModulus(sparkMaxAngle, -180, 180);
    }
/*
     * Change an angle from [-180, 180) to [0, 360), wrapping negative input angles around.
     * 
     * With setPositionPIDWrappingEnabled, sparkmaxes do this automatically for every setpoint < 0
     */
    private double normalAngleToSparkMax(double normalAngle) {
        return MathUtil.inputModulus(normalAngle, 0, 360);
    }

    // TODO: Should this be implemented as a command factory?
    public void setDesired(double fwd, double str, double rcw) {
        this.fwd = fwd;
        this.str = str;
        this.rcw = rcw;
    }

    public void periodic() {
        // When the robot is all width, L = 0, W = R -> rcw points straight back
        var rcw_fwd = rcw * (W / R);
        // When the robot is all length, L = R, W = 0 -> rcw points straight right
        var rcw_str = rcw * (L / R);

        // Front Left
        var fl_fwd = fwd + rcw_fwd;
        var fl_str = str + rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var fl_speed = Math.sqrt(fl_fwd * fl_fwd + fl_str * fl_str);
        var fl_angle = Math.toDegrees(Math.atan2(fl_str, fl_fwd));
        // Front Right
        var fr_fwd = fwd - rcw_fwd;
        var fr_str = str + rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var fr_speed = Math.sqrt(fr_fwd * fr_fwd + fr_str * fr_str);
        var fr_angle = Math.toDegrees(Math.atan2(fr_str, fr_fwd));
        // Rear Left
        var rl_fwd = fwd + rcw_fwd;
        var rl_str = str - rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var rl_speed = Math.sqrt(rl_fwd * rl_fwd + rl_str * rl_str);
        var rl_angle = Math.toDegrees(Math.atan2(rl_str, rl_fwd));
        // Rear Right
        var rr_fwd = fwd - rcw_fwd;
        var rr_str = str - rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var rr_speed = Math.sqrt(rr_fwd * rr_fwd + rr_str * rr_str);
        var rr_angle = Math.toDegrees(Math.atan2(rr_str, rr_fwd));

        var max_output = List.of(fl_speed, fr_speed, rl_speed, rr_speed).stream().max((a, b) -> (int) (a - b))
                .orElse(1.0);
        if (max_output > 1) {
            fl_speed /= max_output;
            fr_speed /= max_output;
            rl_speed /= max_output;
            rr_speed /= max_output;
        }

        // Use this incredibly long formulation of setting duty cycle so that we can see
        // errors from the sparkmaxen
        Util.handleREVLibErr(FLDrive.getPIDController().setReference(fl_speed, ControlType.kDutyCycle));
        Util.handleREVLibErr(FRDrive.getPIDController().setReference(fr_speed, ControlType.kDutyCycle));
        Util.handleREVLibErr(RLDrive.getPIDController().setReference(rl_speed, ControlType.kDutyCycle));
        Util.handleREVLibErr(RRDrive.getPIDController().setReference(rr_speed, ControlType.kDutyCycle));

        // Don't change steering angle when computed speed is 0 - Math.atan2 will
        // snap the angle back to 0 when both inputs are zero. Some people prefer to
        // employ this logic when commanded drive speed is close to zero, I'm of the
        // thought that deadbanding should happen on the input side only, not output.
        if (fl_speed == 0) {
            fl_angle = sparkMaxAngleToNormal(FLSteer.getEncoder().getPosition());
        }
        if (fr_speed == 0) {
            fr_angle = sparkMaxAngleToNormal(FRSteer.getEncoder().getPosition());
        }
        if (rl_speed == 0) {
            rl_angle = sparkMaxAngleToNormal(RLSteer.getEncoder().getPosition());
        }
        if (rr_speed == 0) {
            rr_angle = sparkMaxAngleToNormal(RRSteer.getEncoder().getPosition());
        }

        // TODO - implement wraparound to minimize wheel direction changes
        Util.handleREVLibErr(FLSteer.getPIDController().setReference(normalAngleToSparkMax(fl_angle), ControlType.kPosition));
        Util.handleREVLibErr(FRSteer.getPIDController().setReference(normalAngleToSparkMax(fr_angle), ControlType.kPosition));
        Util.handleREVLibErr(RLSteer.getPIDController().setReference(normalAngleToSparkMax(rl_angle), ControlType.kPosition));
        Util.handleREVLibErr(RRSteer.getPIDController().setReference(normalAngleToSparkMax(rr_angle), ControlType.kPosition));
    }
}
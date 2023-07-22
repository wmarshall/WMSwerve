package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Util;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveDriveTrain implements Subsystem {

    public static double WHEEL_BASE_INCHES = 18;
    public static double TRACK_WIDTH_INCHES = 24;

    public static double L = WHEEL_BASE_INCHES / 2;
    public static double W = TRACK_WIDTH_INCHES / 2;
    public static double R = Math.sqrt(L * L + W * W);

    private final CANSparkMax FLDrive, FLSteer;
    private final CANSparkMax FRDrive, FRSteer;
    private final CANSparkMax RLDrive, RLSteer;
    private final CANSparkMax RRDrive, RRSteer;

    private double fwd, str, rcw;

    private CANSparkMax steerFactory(int deviceID) {
        var steer = new CANSparkMax(deviceID, MotorType.kBrushless);
        // Assume NEO550
        steer.setSmartCurrentLimit(30);
        var heading_encoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
        var motor_encoder = steer.getEncoder();
        // Heading in degrees, 0 is "north"
        Util.handleREVLibErr(heading_encoder.setPositionConversionFactor(360));
        Util.handleREVLibErr(motor_encoder.setPositionConversionFactor(360));

        // TODO - persistent offsets
        Util.handleREVLibErr(heading_encoder.setZeroOffset(0));
        Util.handleREVLibErr(motor_encoder.setPosition(heading_encoder.getPosition()));

        // Configire PID
        var controller = steer.getPIDController();
        Util.handleREVLibErr(controller.setFeedbackDevice(heading_encoder));
        Util.handleREVLibErr(controller.setPositionPIDWrappingEnabled(true));
        Util.handleREVLibErr(controller.setPositionPIDWrappingMinInput(-180));
        Util.handleREVLibErr(controller.setPositionPIDWrappingMaxInput(180));
        // In units of output fraction (-1, 1) per degree of error
        Util.handleREVLibErr(controller.setP(0.05));
        // Enable the controller
        Util.handleREVLibErr(controller.setReference(0, ControlType.kPosition));

        return steer;
    }

    public SwerveDriveTrain() {
        FLDrive = new CANSparkMax(0, MotorType.kBrushless);
        FLSteer = steerFactory(1);
        FRDrive = new CANSparkMax(2, MotorType.kBrushless);
        FRSteer = steerFactory(3);
        RLDrive = new CANSparkMax(4, MotorType.kBrushless);
        RLSteer = steerFactory(5);
        RRDrive = new CANSparkMax(6, MotorType.kBrushless);
        RRSteer = steerFactory(7);
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

        Util.handleREVLibErr(FLDrive.getPIDController().setReference(fl_speed, ControlType.kDutyCycle));
        Util.handleREVLibErr(FRDrive.getPIDController().setReference(fr_speed, ControlType.kDutyCycle));
        Util.handleREVLibErr(RLDrive.getPIDController().setReference(rl_speed, ControlType.kDutyCycle));
        Util.handleREVLibErr(RRDrive.getPIDController().setReference(rr_speed, ControlType.kDutyCycle));

        // Don't change steering angle when computed speed is 0 - Math.atan2 will
        // snap the angle back to 0 when both inputs are zero. Some people prefer to
        // employ this logic when commanded drive speed is close to zero, I'm of the
        // thought that deadbanding should happen on the input side only, not output.
        if(fl_speed == 0) {
            fl_angle = FLSteer.getEncoder().getPosition();
        }
        if(fr_speed == 0) {
            fr_angle = FRSteer.getEncoder().getPosition();
        }
        if(rl_speed == 0) {
            rl_angle = RLSteer.getEncoder().getPosition();
        }
        if(rr_speed == 0) {
            rr_angle = RRSteer.getEncoder().getPosition();
        }

        // TODO - implement wraparound to minimize wheel direction changes

        // Use this incredibly long formulation of setting duty cycle so that we can see
        // errors from the sparkmaxen
        Util.handleREVLibErr(FLSteer.getPIDController().setReference(fl_angle, ControlType.kPosition));
        Util.handleREVLibErr(FRSteer.getPIDController().setReference(fr_angle, ControlType.kPosition));
        Util.handleREVLibErr(RLSteer.getPIDController().setReference(rl_angle, ControlType.kPosition));
        Util.handleREVLibErr(RRSteer.getPIDController().setReference(rr_angle, ControlType.kPosition));
    }
}
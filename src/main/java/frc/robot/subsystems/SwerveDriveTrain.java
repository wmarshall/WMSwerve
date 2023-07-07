package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveDriveTrain implements Subsystem {

    public static double WHEEL_BASE_INCHES = 18;
    public static double TRACK_WIDTH_INCHES = 24;

    public static double L = WHEEL_BASE_INCHES /2;
    public static double W = TRACK_WIDTH_INCHES /2;
    public static double R = Math.sqrt(L*L + W*W);

    private final CANSparkMax FLDrive, FLSteer;
    private final CANSparkMax FRDrive, FRSteer;
    private final CANSparkMax RLDrive, RLSteer;
    private final CANSparkMax RRDrive, RRSteer;

    private double fwd, str, rcw;

    private CANSparkMax steerFactory(int deviceID) {
        var steer = new CANSparkMax(deviceID, MotorType.kBrushless);
        // Assume NEO550
        steer.setSmartCurrentLimit(30);
        var encoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
        // TODO - persistent offsets
        encoder.setZeroOffset(0);

        // Configire PID
        var controller = steer.getPIDController();
        controller.setFeedbackDevice(encoder);
        controller.setPositionPIDWrappingEnabled(true);
        // TODO - what units is this operating in
        controller.setP(1);
        // Enable the controller
        controller.setReference(0, ControlType.kPosition);

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

    public void setDesired(double fwd, double str, double rcw) {
        this.fwd = fwd;
        this.str = str;
        this.rcw = rcw;
    }

    public void periodic(){
        // When the robot is all width, L = 0, W = R -> rcw points straight back
        var rcw_fwd = rcw * (W/R);
        // When the robot is all length, L = R, W = 0 -> rcw points straight right

        var rcw_str = rcw * (L/R);
        
        // Front Left
        var fl_fwd = fwd + rcw_fwd;
        var fl_str = str + rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var fl_speed = Math.sqrt(fl_fwd*fl_fwd + fl_str*fl_str);
        var fl_angle = Math.atan2(fl_str, fl_fwd);
        // Front Right
        var fr_fwd = fwd - rcw_fwd;
        var fr_str = str + rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var fr_speed = Math.sqrt(fr_fwd*fr_fwd + fr_str*fr_str);
        var fr_angle = Math.atan2(fr_str, fr_fwd);
        // Rear Left
        var rl_fwd = fwd + rcw_fwd;
        var rl_str = str - rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var rl_speed = Math.sqrt(rl_fwd*rl_fwd + rl_str*rl_str);
        var rl_angle = Math.atan2(rl_str, rl_fwd);
        // Rear Right
        var rr_fwd = fwd - rcw_fwd;
        var rr_str = str - rcw_str;
        // Assumes instantaneous angle correction - may cause lurch in wrong direction
        var rr_speed = Math.sqrt(rr_fwd*rr_fwd + rr_str*rr_str);
        var rr_angle = Math.atan2(rr_str, rr_fwd);

        // TODO - normalize speeds s.t. we're not commanding >100% output
        // TODO - don't change steering angle when computed speed is 0 - Math.atan2 will snap the angle back to 0 when both inputs are zero
        // TODO - implement wraparound to minimize wheel direction changes

        FLDrive.set(fl_speed);
        // TODO - assumes angle in radians?
        FLSteer.getPIDController().setReference(fl_angle, ControlType.kPosition);
        
        FRDrive.set(fr_speed);
        // TODO - assumes angle in radians?
        FRSteer.getPIDController().setReference(fr_angle, ControlType.kPosition);

        RLDrive.set(rl_speed);
        // TODO - assumes angle in radians?
        RLSteer.getPIDController().setReference(rl_angle, ControlType.kPosition);

        RRDrive.set(rr_speed);
        // TODO - assumes angle in radians?
        RRSteer.getPIDController().setReference(rr_angle, ControlType.kPosition);

    }
}
package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .7142857143; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.924293944230984;//(14.909127039765096*2 + 14.87526010741)/3.0;//(14.841434054082098); //14.913514392739817; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel
    public static double X_MULTIPLIER = .9743357717054657;
    public static double Y_MULTIPLIER = .971723797796964; //.979301610020459;  0.971038477101266

    public static double R_ERROR = 0; //3

    public static double R_MULTIPLIER = (3600.0)/(3600.0+R_ERROR);

    private Encoder leftEncoder, rightEncoder, frontEncoder;
//
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "outer_roller1"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fly_wheel"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "outer_roller"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition())*X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition())*X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition())*Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity())*X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity())*X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())*Y_MULTIPLIER
        );
    }
}

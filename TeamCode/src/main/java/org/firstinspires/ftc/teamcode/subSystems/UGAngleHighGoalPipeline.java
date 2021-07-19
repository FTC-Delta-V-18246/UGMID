package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilnonrr.PIDMath;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
@Config
public class UGAngleHighGoalPipeline extends UGBasicHighGoalPipeline {

    class Fraction {
        private int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

    // Camera Settings
    protected int imageWidth;
    protected int imageHeight;

    private double cameraPitchOffset;
    private double cameraYawOffset;

    public static double fov = 78;
    public static double XCenter = 160;
    private double horizontalFocalLength;
    private double verticalFocalLength;
    public Telemetry telemetry;
    public static double kP = .02, kD = 0, kF = 1;
    private PIDMath turnController = new PIDMath(kP, 0, kD, kF);
    public static double angleTolerance = 2;
    public enum Target {
        RED, BLUE
    }

    public UGAngleHighGoalPipeline(LinearOpMode opMode) {
        super();
        telemetry = opMode.telemetry;
        this.cameraPitchOffset = cameraPitchOffset;
        this.cameraYawOffset = cameraYawOffset;
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        imageWidth = mat.width();
        imageHeight = mat.height();

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }

    @Override
    public Mat processFrame(Mat input) {
        input = super.processFrame(input);
        return input;
    }

    /**
     * @param color Alliance Color
     */
    public double calculateYaw(Target color) {
        return calculateYaw(color, centerX) + cameraYawOffset;
    }

    /**
     * @param color Alliance Color
     */
    public double calculatePitch(Target color) {
        return calculatePitch(color, centerY) + cameraPitchOffset;
    }

    /**
     * @param color         Alliance color
     * @param offsetCenterX centerX
     */
    public double calculateYaw(Target color, double offsetCenterX) {
        Rect currentRect = color == Target.RED ? getRedRect() : getBlueRect();
        double targetCenterX = getCenterofRect(currentRect).x;
        return Math.toDegrees(
                Math.atan((targetCenterX - XCenter) / horizontalFocalLength)
        );
    }

    /**
     * @param color         Alliance color
     * @param offsetCenterY centerY
     */
    public double calculatePitch(Target color, double offsetCenterY) {
        Rect currentRect = color == Target.RED ? getRedRect() : getBlueRect();
        double targetCenterY = getCenterofRect(currentRect).y;

        return -Math.toDegrees(
                Math.atan((targetCenterY - offsetCenterY) / verticalFocalLength)
        );
    }
    public double angleAlign(Target color){
        double angle = calculateYaw(color,XCenter);
        turnController.PIDConstants(kP,0,kD,kF);
        if(Math.abs(calculateYaw(color))>angleTolerance) {
            return Range.clip(turnController.calculateGain(angle),-1,1);
        }
        return 0;
    }

}
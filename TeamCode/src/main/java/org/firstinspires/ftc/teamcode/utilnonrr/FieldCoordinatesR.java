package org.firstinspires.ftc.teamcode.utilnonrr;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.geometry.Point;

public class FieldCoordinatesR {
    public Point HL, HR;
    public static double align = -36;
    public static Point HM = new Point(72,-36);
    public static Point PL = new Point(72, -26);
    public static Point PM = new Point(72, -19);
    public static Point PR = new Point(72, -11);
    public static Pose2d WAI = new Pose2d(-12, -58);
    public static Pose2d WAII = new Pose2d(-12, -60, 0);
    public static Pose2d WBI = new Pose2d(18, -39);
    public static Pose2d WBII = new Pose2d(15, -28, 0);
    public static Pose2d WCI = new Pose2d(38, -65);
    public static Pose2d WCII = new Pose2d(34, -63, 0);
    public static Pose2d WPU  =  new Pose2d(-28.5, -28, Math.PI);
    public static Pose2d PAL = new Pose2d(4,-12);
    public static Pose2d PAM = new Pose2d(9,-36);
    public static Pose2d PAR = new Pose2d(9,-60);
    public static Pose2d PRLB = new Pose2d(-16, -12,0);
    public static Pose2d PRLC = new Pose2d(-14, align,0);
    public static Pose2d PS = new Pose2d(-18,-18,0);
    public FieldCoordinatesR()
    {
        HR = new Point(72,-42);
        HL = new Point(72,-26);
    }
}

package org.firstinspires.ftc.teamcode.utilnonrr;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.geometry.Point;

@Config
public class FieldCoordinatesB {
    public Point HL, HR;
    public static double align = 38;
    public static Point HM = new Point(72,36);
    public static Point PL = new Point(72, 0);
    public static Point PM = new Point(72, 6);
    public static Point PR = new Point(72, 12);
    public static Pose2d WAI = new Pose2d(-13, 58,-7.0*Math.PI/8.0);
    public static Pose2d WAIB = new Pose2d(35, 56,0);
    public static Pose2d WAII = new Pose2d(-12, 60, Math.PI);
    public static Pose2d WBI = new Pose2d(13, 53,-5.0*Math.PI/4.0);
    public static Pose2d WBIB = new Pose2d(35, 13,-Math.PI/2.0);
    public static Pose2d WBII = new Pose2d(15, 32, Math.PI);
    public static Pose2d WCI = new Pose2d(31, 63,Math.PI);
    public static Pose2d WCIB = new Pose2d(40, 42,-5.0*Math.PI/6.0);
    public static Pose2d WCII = new Pose2d(32, 63, Math.PI);
    public static Pose2d WPU  =  new Pose2d(-30, 24, Math.PI);
    public static Pose2d PAL = new Pose2d(7,15,0);
    public static Pose2d PAM = new Pose2d(2,36,0);
    public static Pose2d PAR = new Pose2d(2,60,Math.PI);
    public static Pose2d PRLB = new Pose2d(-16, 14,0);
    public static Pose2d PRLC = new Pose2d(-38, align,0);
    public static Pose2d PS = new Pose2d(-10,18,0);

    public FieldCoordinatesB()
    {
        //HM = new Point(72,32);
        HL = new Point(72,42);
        HR = new Point(72,26);
    }
}

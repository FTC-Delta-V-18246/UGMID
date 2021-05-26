package org.firstinspires.ftc.teamcode.utilnonrr;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.geometry.Point;

@Config
public class FieldCoordinatesB {
    public Point HL, HR;
    public static Point HM = new Point(72,36);
    public static Point PL = new Point(72, 26);
    public static Point PM = new Point(72, 19);
    public static Point PR = new Point(72, 11);
    private double toInches = 1/25.4;
    public FieldCoordinatesB()
    {
        //HM = new Point(72,32);
        HL = new Point(72,42);
        HR = new Point(72,26);
    }
}

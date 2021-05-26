package org.firstinspires.ftc.teamcode.subSystems.retired;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;

@Disabled
public class revIMU
{


    LinearOpMode opModeObj;
    BNO055IMU imu;

    public revIMU(LinearOpMode opmode, hardwareGenerator gen){
        /*imu = gen.imu;*/
    }

    public double getAngle(){
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return heading;

    }
    public double posDif(double currPos, double endPos){
        return Math.abs(currPos-endPos);

    }



}

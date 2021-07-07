package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class reader {

    LinearOpMode opModeObj;
    hardwareGenerator gen;
    SampleMecanumDrive driver;

    public double outerRollerV, innerRollerV, shooterV;
    BNO055IMU imu;

    double shooterPos;
    Orientation angles;
    public Pose2d curPose;
    public List<LynxModule> allHubs;
    private double loops = 0;

    public reader(LinearOpMode opMode, hardwareGenerator hard, SampleMecanumDrive driver, ElapsedTime timer){
        opModeObj = opMode;

         allHubs = opModeObj.hardwareMap.getAll(LynxModule.class);

        bulkManual();
        gen = hard;
        this.driver = driver;
        opModeObj = opMode;
    }

    public void autonRead(){
        bulkManualClear();
        veloRead();
        odoRead();

    }
    public void teleRead(){
        bulkManualClear();
        veloRead();
        odoRead();

    }

    public void bulkManual(){

        for (LynxModule module : allHubs) {
         //   module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


    }
    public void bulkManualClear() {

        for (LynxModule module : opModeObj.hardwareMap.getAll(LynxModule.class)) {
         //   module.clearBulkCache();
        }

    }
    public void veloRead(){
        //outerRollerV = -gen.outerRollerM.getVelocity(AngleUnit.RADIANS)*8*.036;
        //innerRollerV = gen.innerRollerM.getVelocity(AngleUnit.RADIANS)*8*.018;
        shooterV = -gen.flyWheelM1.getVelocity()/(28.0)*1.378597428*.096* Math.PI;
    }
    public void controllerRead(){

    }
    public void odoRead(){
        /*
        loops++;
        if(loops>=20){
            loops = 0;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if(angles.firstAngle<0){
                angles.firstAngle+=2*Math.PI;
            }
            driver.setPoseEstimate(new Pose2d(driver.getPoseEstimate().getX(),driver.getPoseEstimate().getY(),angles.firstAngle+Math.PI));
        }
        */
        curPose = driver.getPoseEstimate();
    }
}

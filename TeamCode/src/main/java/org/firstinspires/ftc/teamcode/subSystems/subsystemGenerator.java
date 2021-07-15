package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.retired.bulk;
import org.firstinspires.ftc.teamcode.subSystems.retired.drive;

@Config
public class subsystemGenerator {

    public drive vroomer;
    public bulk reader;
    public hood shooter;
    public org.firstinspires.ftc.teamcode.subSystems.reader hardReader;
    public intake roller; //
    public hardwareGenerator support;
    public SampleMecanumDrive driver;
    public vision camera;
    public wobble hammer;
    public tracker magTrak;
    public static double fireSpeed = 19;


    public subsystemGenerator(LinearOpMode opMode, hardwareGenerator hard, ElapsedTime timer){
        support = hard;
        magTrak = new tracker(hard, timer);
      //  align = new high(opMode);
        camera = new vision(opMode);
        shooter = new hood(opMode, support, fireSpeed,.5,0,0,.02 , magTrak, camera);   // .4, .05, .07, .0299
        driver = new SampleMecanumDrive(opMode.hardwareMap);
        hardReader = new reader(opMode, hard, driver, timer);
        vroomer = new drive(opMode, hard, driver, shooter, timer);
        roller = new intake(opMode,support);
        hammer = new wobble(opMode,support);




    }

}

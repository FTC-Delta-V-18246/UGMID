package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.hood;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.reader;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision;
import org.firstinspires.ftc.teamcode.utilnonrr.ButtonReader;
import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesB;
import org.firstinspires.ftc.teamcode.utilnonrr.GamepadEx;
import org.firstinspires.ftc.teamcode.utilnonrr.GamepadKeys;

//import com.acmerobotics.dashboard.config.Config;

//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;


@TeleOp
@Config
public class wraithtester extends LinearOpMode {

    public enum RobotState {
        INDEXING,
        POWER,
        HIGH,
        WOBBLE,
    }
    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public static double fireAngle = 30;
    public static double offset = -.1;
    public static double pos =.3 ;
    public static double targetAngleCustom = Math.toRadians(30);
    //testing
    //public static double topRectHeightPercentage = 0.45, topRectWidthPercentage = 0.45, bottomRectHeightPercentage = 0.45, bottomRectWidthPercentage = 0.55;
    //public static int rectangleWidth = 10, rectangleHeight = 10;

    private ElapsedTime runtime = new ElapsedTime();

    boolean left_POWER = false, right_POWER = false, middle_POWER = false;

    /**
     * adding vision
     */

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() {

        RobotState shooterState = RobotState.INDEXING;

        hardwareGenerator gen = new hardwareGenerator(this);
        subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);

        // subs.ringGen.init();


        this.driver = subs.driver;
        this.hardReader = subs.hardReader;
        this.shooter = subs.shooter;
        this.roller = subs.roller;
        this.camera = subs.camera;
        FieldCoordinatesB field = new FieldCoordinatesB();

        telemetry.addData("Init", "Successful");
        telemetry.update();

        waitForStart();
        runtime.reset();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        ButtonReader x1 = new ButtonReader(gamepadEx, GamepadKeys.Button.X);
        ButtonReader b1 = new ButtonReader(gamepadEx, GamepadKeys.Button.B);
        ButtonReader y1 = new ButtonReader(gamepadEx, GamepadKeys.Button.Y);
        ButtonReader a1 = new ButtonReader(gamepadEx, GamepadKeys.Button.A);
        gen. frontLeftM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen. frontRightM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen. backLeftM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen. backRightM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        double goodTime = 0;
        double badTime = 0;
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.teleRead();
            x1.readValue();
            b1.readValue();
            y1.readValue();
            a1.readValue();

            /*
            if(gamepad1.b){
                telemetry.addData("Stack",camera.height());
                gen.outerRollerMII.setPower(0);
                gen.outerRollerMI.setPower(0);
            }else{
                gen.outerRollerMII.setPower(1);
                gen.outerRollerMI.setPower(0);
            }
*/
            if(gamepad1.dpad_left){
                driver.turnAsync(field.PL);
            }else if(gamepad1.dpad_right){
                driver.turnAsync(field.PR);
            }
            else if(gamepad1.dpad_down){
                driver.turnAsync(field.PM);
            }else if(gamepad1.dpad_up){
                driver.setPoseEstimate(new Pose2d(0,0,0));
            }
            shooter.upToSpeed(hardReader.shooterV,runtime.seconds());
            if(gamepad1.right_bumper){
                shooter.fire(hardReader.shooterV);
            }else{
                shooter.timedCancel();
            }
            /*
            if(gamepad1.b&&!shooter.done){
                shooter.timedFireN(hardReader.shooterV);
                roller.upToSpeed(0);
            }else if(gamepad1.a){
                shooter.timedCancel();
               // roller.upToSpeed();
            }else
            {
                roller.upToSpeed(0);
            }
            if(!gamepad1.b){
                shooter.done = false;
                shooter.timedCancel();
            }*/
/*
           roller.upToSpeed();
            if(gamepad1.b){
                subs.magTrak.tunerReset();
                shooter.liftDown();
            }
            //driver.update();
            roller.fallOut();
            telemetry.update();
            if(subs.magTrak.counter(hardReader.curX, hardReader.curV)>2){
                shooter.liftUp();
            }else{
                //shooter.toPosition(.22);
            }

 */
            shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,true));
            subs.hammer.partialLift();
            shooter.liftUp();
            driver.update();
            FtcDashboard dashboard = FtcDashboard.getInstance();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("rings in canister", subs.magTrak.counter(hardReader.curX, hardReader.curV));
            packet.put("ring pos", hardReader.curX);
            packet.put("ring velo",hardReader.curV);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}


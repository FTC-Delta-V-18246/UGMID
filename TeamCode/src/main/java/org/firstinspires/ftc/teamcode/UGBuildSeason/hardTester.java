package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
public class hardTester extends LinearOpMode {

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
            if(gamepad1.a){
                gen.flapServo.setPosition(pos);
            }
            if(gamepad1.b){
                gen.wobbleClaw.setPosition(pos);
            }
            if(gamepad1.x){
                gen.wobblePivot.setPosition(pos);
            }
            if(gamepad1.y){
                gen.liftServo.setPosition(pos);
            }
            if(gamepad2.a){
                gen.outerRollerMI.setPower(pos);
            }
            else if(gamepad2.b){
                gen.outerRollerMII.setPower(pos);
            }
            else if(gamepad2.x){
                gen.flyWheelM.setPower(pos);
                gen.flyWheelM1.setPower(pos);
            }
            else if(gamepad2.y){

            }
            else {
                gen.flyWheelM1.setPower(0);
                gen.flyWheelM.setPower(0);
                gen.outerRollerMI.setPower(0);
                gen.outerRollerMII.setPower(0);


            }
        }
    }
}


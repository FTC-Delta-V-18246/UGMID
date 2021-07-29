package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.MTI.FieldCoordinatesB;
import org.firstinspires.ftc.teamcode.utilnonrr.GamepadEx;
import org.firstinspires.ftc.teamcode.utilnonrr.GamepadKeys;

//import com.acmerobotics.dashboard.config.Config;

//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

@TeleOp
@Config
public class thermometer extends LinearOpMode {

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
    public static double pos = 0;
    public static double targetAngleCustom = Math.toRadians(30);
    //testing
    //public static double topRectHeightPercentage = 0.45, topRectWidthPercentage = 0.45, bottomRectHeightPercentage = 0.45, bottomRectWidthPercentage = 0.55;
    //public static int rectangleWidth = 10, rectangleHeight = 10;

    private ElapsedTime runtime = new ElapsedTime();
public static double whatever= 180;

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
        roller.fallOut();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        ButtonReader x1 = new ButtonReader(gamepadEx, GamepadKeys.Button.X);
        ButtonReader b1 = new ButtonReader(gamepadEx, GamepadKeys.Button.B);
        ButtonReader y1 = new ButtonReader(gamepadEx, GamepadKeys.Button.Y);
        ButtonReader a1 = new ButtonReader(gamepadEx, GamepadKeys.Button.A);
        gen. frontLeftM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen. frontRightM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen. backLeftM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen. backRightM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gen . backRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double goodTime = 0;
        double badTime = 0;
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.teleRead();
            x1.readValue();
            b1.readValue();
            y1.readValue();
            a1.readValue();
            telemetry.addData("Pos", gen.backRightM.getCurrentPosition());
            /*
            if(gamepad1.right_bumper){

                shooter.fire(hardReader.shooterV);
            }
            else if(gamepad1.left_bumper){
                subs.angler.toPosition(.8);
                gen.pusherServo.setPosition(shooter.leftPusherPos);
                roller.upToSpeedI();
                roller.upToSpeedO();
            }else{
                subs.angler.toPosition(whatever);
            }
*/
            /*
            if(x1.wasJustPressed()){
                //driver.turnAsync(Math.toRadians(whatever));
                //driver.turnAsync(field.HM);
            }
            if(y1.wasJustPressed()){
                //driver.turnAsync(field.HL);
                //driver.turnAsync(field.PM);
            }
            driver.update();
            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard dashboard = FtcDashboard.getInstance();
            packet.put("Target Velocity", subs.fireSpeed);
            // packet.put("Actual Velocity Outer", hardReader.outerRollerV);
            // packet.put("Actual Velocity Inner", hardReader.innerRollerV);
            packet.put("Actual Velocity Shooter", hardReader.shooterV);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            /*
            if(gamepad1.a) {
                shooter.feed();
            }
            else{
                shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.PM,hardReader.curPose,true));
            }
            //subs.hammer.grab();
            //shooter.adjusterToAngle(Math.toRadians(fireAngle));
            if(gamepad1.left_bumper){
                shooter.upToSpeed(hardReader.shooterV,runtime.seconds());
            }
            else{
                shooter.safetySwitch(hardReader.shooterV,runtime.seconds());
            }

            if(gamepad1.b){
                shooter.fire(hardReader.shooterV);
                roller.upToSpeedI(0);
                roller.upToSpeedO(0);
            }
            else{
                roller.upToSpeedI();
                roller.upToSpeedO();
            }
            if (gamepad1.dpad_up) {
                driver.turn(shooter.turnToAbsolute(field.PM,hardReader.curPose));
            }else{
                driver.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -.8*gamepad1.right_stick_x
                        )
                );
            }*/

            /*
            if(gamepad1.x){
                driver.turn(shooter.turnToAbsolute(field.PL,hardReader.curPose));
            }
            else if(gamepad1.y){
                driver.turn(shooter.turnToAbsolute(field.PM,hardReader.curPose));
            }
            else if(gamepad1.b){
                driver.turn(shooter.turnToAbsolute(field.PR,hardReader.curPose));
            }


            driver.update();
            telemetry.addData("Tan",shooter.turnToAbsolute(field.PM,hardReader.curPose));
            telemetry.addData("Limit",shooter.ejectorAtRest());
            telemetry.addData("Velo",shooter.atSpeed(hardReader.shooterV));
            telemetry.addData("Rings",subs.camera.height());
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard dashboard = FtcDashboard.getInstance();
            packet.put("Target Velocity", subs.fireSpeed);
            // packet.put("Actual Velocity Outer", hardReader.outerRollerV);
            // packet.put("Actual Velocity Inner", hardReader.innerRollerV);
            packet.put("Actual Velocity Shooter", hardReader.shooterV);

            dashboard.sendTelemetryPacket(packet);
             */
            /*
            if(b1.wasJustPressed()){
                pos+=.025;
            }
            if(a1.wasJustPressed()){
                pos-=.025;
            }
            subs.angler.toPosition(pos);

            if(gamepad1.left_bumper){
                subs.hammer.lift();
            }else{
                subs.hammer.down();
            }
            if(gamepad1.right_bumper){
                subs.hammer.grab();
            }else{
                subs.hammer.release();
            }
            */

            if(y1.isDown()){
              // gen.pusherServo.setPosition(hood.rightPusherPos);
            }else{
              // gen.pusherServo.setPosition(hood.leftPusherPos);
            }
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", subs.fireSpeed);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            packet.put("Actual Velocity Outer", hardReader.outerRollerV);
            packet.put("Actual Velocity Inner", hardReader.innerRollerV);
            packet.put("Actual Velocity Shooter", hardReader.shooterV);
            dashboard.sendTelemetryPacket(packet);
            shooter.upToSpeed(hardReader.shooterV,runtime.seconds());

            if (b1.isDown()) {
                shooter.fire(hardReader.shooterV);
            }else if (!b1.isDown()){
                gen.pusherServo.setPosition(hood.leftPusherPos);
                telemetry.addData("Target Angle", shooter.calculateTargetShooterAngle(field.HM,hardReader.curPose, false));
            }
           // roller.upToSpeed();

            /*
            shooter.upToSpeed(hardReader.shooterV,runtime.seconds());
            if(x1.wasJustPressed()){
                driver.turnAsync(Math.toRadians(whatever));
                //driver.turnAsync(field.HM);
            }
            if(y1.wasJustPressed()){
                driver.turnAsync(field.HL);
                //driver.turnAsync(field.PM);
            }
            //driver.setWeightedDrivePower(new Pose2d(0,0,driver.rotation.calculateGain(targetAngleCustom - driver.getPoseEstimate().getHeading(), runtime.seconds())));
          // driver.update();
            // gen.flyWheelM.setPower(.3);
           // gen.flyWheelM1.setPower(.3);
            telemetry.addData("bad time", badTime);
            telemetry.addData("flywheel power",gen.flyWheelM.getPower());
            telemetry.addData("pos",pos);
            telemetry.update();
            FtcDashboard dashboard = FtcDashboard.getInstance();
            if(shooter.atSpeed(hardReader.shooterV)){
                goodTime = runtime.seconds();
            }else{
                badTime = runtime.seconds()-goodTime;
            }



            /*
            subs.angler.toPosition(pos);

            if(y1.isDown()){
                shooter.upToSpeed(hardReader.shooterV,runtime.seconds());
                shooter.fire(hardReader.shooterV);
                roller.upToSpeedO(0);
                roller.upToSpeedI(0);
            } else{
                roller.upToSpeedO();
                roller.upToSpeedI();
                shooter.safetySwitch(hardReader.shooterV,runtime.seconds());
            }

             */
        }
    }
}


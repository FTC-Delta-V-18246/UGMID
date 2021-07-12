  package org.firstinspires.ftc.teamcode.MTI;

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

 import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
 import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
 import org.firstinspires.ftc.teamcode.subSystems.PoseStorage;
 import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
 import org.firstinspires.ftc.teamcode.subSystems.hood;
 import org.firstinspires.ftc.teamcode.subSystems.intake;
 import org.firstinspires.ftc.teamcode.subSystems.reader;
 import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
 import org.firstinspires.ftc.teamcode.utilnonrr.ButtonReader;
 import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesB;
 import org.firstinspires.ftc.teamcode.utilnonrr.GamepadEx;
 import org.firstinspires.ftc.teamcode.utilnonrr.GamepadKeys;

 @TeleOp
 @Config
 public class BTele extends LinearOpMode {

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
     boolean turning = false;
     boolean autoAngle = true;
     public static RobotState shooterState = RobotState.INDEXING;;
     private ElapsedTime runtime = new ElapsedTime();

     @RequiresApi(api = Build.VERSION_CODES.N)
     public void runOpMode() {


         hardwareGenerator gen = new hardwareGenerator(this);
         subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);

         gen.frontLeftM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
         gen.frontRightM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
         gen.backLeftM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
         gen.backRightM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

         this.driver = subs.driver;
         this.hardReader = subs.hardReader;
         this.shooter = subs.shooter;
         this.roller = subs.roller;
         FieldCoordinatesB field = new FieldCoordinatesB();
         driver.setPoseEstimate(PoseStorage.currentPose);
         telemetry.addData("Init", "Successful");
         telemetry.update();

         waitForStart();
         runtime.reset();
         roller.fallOut();
         subs.hammer.grab();
         subs.hammer.lift();
         GamepadEx gamepadEx = new GamepadEx(gamepad1);
         ButtonReader x1 = new ButtonReader(gamepadEx, GamepadKeys.Button.X);
         ButtonReader b1 = new ButtonReader(gamepadEx, GamepadKeys.Button.B);
         ButtonReader y1 = new ButtonReader(gamepadEx, GamepadKeys.Button.Y);
         ButtonReader a1 = new ButtonReader(gamepadEx, GamepadKeys.Button.A);
         while (!isStopRequested() && opModeIsActive()) {
             driver.update();
             hardReader.teleRead();
             PoseStorage.currentPose = hardReader.curPose;
             x1.readValue();
             b1.readValue();
             y1.readValue();
             a1.readValue();
             switch (shooterState) {
                 case INDEXING:
                    // shooter.feed();
                     shooter.liftDown();
                     subs.hammer.lift();
                     gen.pusherServo.setPosition(hood.leftPusherPos);
                     shooter.timedCancel();
                     break;

                 case HIGH:
                     shooter.liftUp();
                     if(autoAngle) {
                         shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose, false));
                     }else{
                         shooter.raiseToAngle(shooter.levelFlap);
                     }
                     if (!driver.isBusy() && gamepad1.right_bumper) {
                         shooter.timedFireN(hardReader.shooterV);
                     }else {
                         shooter.timedCancel();
                     }
                     if(gamepad1.left_bumper){
                         driver.turnAsync(field.HM);
                         turning = true;
                     }else{
                         turning = false;
                         driver.cancelFollowing();
                     }

                     break;
                 case POWER:
                     shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.PM, hardReader.curPose,true));
                     if(gamepad1.dpad_down){
                         driver.setPoseEstimate(new Pose2d(0,-24,0));
                     }else if(gamepad1.dpad_left){
                         driver.turnAsync(field.PL);
                         turning = true;
                     }else if(gamepad1.dpad_up){
                         driver.turnAsync(field.PM);
                         turning = true;
                     }else if(gamepad1.dpad_right){
                         driver.turnAsync(field.PR);
                         turning = true;
                     }
                     if(!driver.isBusy()){
                         turning = false;
                     }
                        /*
                         if(turning && !driver.isBusy()&&shooter.atSpeed(hardReader.shooterV)) {
                             turning = false;
                             gen.pusherServo.setPosition(hood.rightPusherPos);
                         }
                        if(driver.isBusy()){
                            turning = true;
                            gen.pusherServo.setPosition(hood.leftPusherPos);
                        }

                      */
                     if (gamepad1.right_bumper) {
                         shooter.fire(hardReader.shooterV);
                     }else {
                         gen.pusherServo.setPosition(hood.leftPusherPos);
                     }

                     break;
                 case WOBBLE:
                     roller.tuckIn();
                     shooter.toPosition(.29);
                     if(gamepad1.left_bumper){
                         subs.hammer.grab();
                     }else{
                         subs.hammer.release();
                     }
                     if(gamepad1.right_bumper){
                         subs.hammer.lift();
                     }else{
                         subs.hammer.down();
                     }
                     break;
             }

            //driver.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
             if(!turning) {
                 driver.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, Math.toDegrees(hardReader.curPose.getHeading())+90);
             }else{
                 //driver.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toDegrees(hardReader.curPose.getHeading()));
             }
             if (shooterState!= RobotState.HIGH&&shooterState!= RobotState.POWER) {
                 shooter.safetySwitch();
             }
             else{
                  shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
             }

             if(!gamepad1.left_bumper&&shooterState== RobotState.INDEXING){
                 roller.fallOut();
                 roller.upToSpeed();
             }
             else if(gamepad1.right_bumper&&shooterState== RobotState.INDEXING){
                 roller.upToSpeed(-.8);
                 roller.tuckIn();
             }
             else{
                 roller.upToSpeed(0);
             }

             if (y1.wasJustPressed()&&shooterState!= RobotState.WOBBLE) {
                 shooterState = RobotState.POWER;
             }
             if (b1.wasJustPressed()&&shooterState!= RobotState.WOBBLE) {
                 shooterState = RobotState.HIGH;
                 // driver.turnAsync(field.HM);

             }
             if (a1.wasJustPressed()) {
                 shooterState = RobotState.WOBBLE;
             }
             if(x1.wasJustPressed()){
                 shooterState = RobotState.INDEXING;
             }
             telemetry.addData("shooter velo", hardReader.shooterV);
             telemetry.addData("Intake amps", gen.outerRollerMI.getCurrent(CurrentUnit.AMPS)+gen.outerRollerMII.getCurrent(CurrentUnit.AMPS));
             telemetry.addData("Shooter amps", gen.flyWheelM.getCurrent(CurrentUnit.AMPS)+gen.flyWheelM1.getCurrent(CurrentUnit.AMPS));
             telemetry.addData("Drive amps", gen.frontRightM.getCurrent(CurrentUnit.AMPS)+gen.frontLeftM.getCurrent(CurrentUnit.AMPS)+gen.backLeftM.getCurrent(CurrentUnit.AMPS)+gen.backRightM.getCurrent(CurrentUnit.AMPS));
             telemetry.addData("x", hardReader.curPose.getX());
             telemetry.addData("y", hardReader.curPose.getY());
             telemetry.addData("heading", hardReader.curPose.getHeading());
             telemetry.addData("State ",shooterState);
             telemetry.update();

             if(gamepad2.b){
                 driver.setPoseEstimate(new Pose2d());
                 autoAngle = false;
             }
             FtcDashboard dashboard = FtcDashboard.getInstance();

             TelemetryPacket packet = new TelemetryPacket();
             packet.put("Target Velocity", subs.fireSpeed);
             packet.put("Actual Velocity Shooter", hardReader.shooterV);

             }
         }
     }


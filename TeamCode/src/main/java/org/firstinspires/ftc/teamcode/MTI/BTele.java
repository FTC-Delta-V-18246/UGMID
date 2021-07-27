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
 import org.firstinspires.ftc.teamcode.subSystems.UGAngleHighGoalPipeline;
 import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
 import org.firstinspires.ftc.teamcode.subSystems.hood;
 import org.firstinspires.ftc.teamcode.subSystems.intake;
 import org.firstinspires.ftc.teamcode.subSystems.reader;
 import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
 import org.firstinspires.ftc.teamcode.subSystems.vision;
 import org.firstinspires.ftc.teamcode.util.wait;
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
     public vision align;
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
         this.align = subs.camera;
         align.highGoal();
         FieldCoordinatesB field = new FieldCoordinatesB();
         driver.setPoseEstimate(PoseStorage.currentPose);
         telemetry.addData("Init", "Successful");
         telemetry.update();

         double cycles = 0;

         wait lift = new wait(1,true);
        lift.init();
         waitForStart();
         runtime.reset();
         roller.fallOut();
         subs.hammer.release();
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
                     shooter.liftDown();
                     subs.hammer.lift();
                     gen.pusherServo.setPosition(hood.leftPusherPos);
                     shooter.timedCancel();
                     if(subs.magTrak.counter(hardReader.curX, hardReader.curV)>2){
                        // shooterState = RobotState.HIGH;
                     }
                     break;

                 case HIGH:
                     subs.hammer.partialLift();
                     shooter.liftUp();
                     if(lift.timeUp()) {
                         if (autoAngle) {
                             //shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose, false));
                             shooter.raiseToAngle(shooter.flapH);
                         } else {
                             shooter.raiseToAngle(shooter.flapH);
                         }
                         if (!driver.isBusy() && gamepad1.right_bumper) {
                             shooter.timedFireN(hardReader.shooterV);
                         } else {
                             shooter.timedCancel();
                         }
                         if (gamepad1.left_bumper) {
                             turning = true;
                         } else {
                             turning = false;
                         }
                         lift.deinit();
                     }
                     if(gamepad1.x){
                         lift.init();
                     }

                     break;
                 case POWER:
                     shooter.liftUp();
                     shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.PM, hardReader.curPose,true));
                     subs.hammer.partialLift();
                     if(gamepad1.dpad_down){
                         driver.setPoseEstimate(new Pose2d(0,0,0));
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
                     lift.deinit();
                     if(gamepad1.x){
                         lift.init();
                     }

                     break;
                 case WOBBLE:
                     roller.tuckIn();
                     shooter.liftDown();
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
             }else if(turning && shooterState == RobotState.HIGH){
                 if(align.goalline.isBlueVisible()) {
                     driver.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, align.goalline.angleAlign(UGAngleHighGoalPipeline.Target.BLUE),Math.toDegrees(hardReader.curPose.getHeading())+90);
                 }else{
                     driver.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, Math.toDegrees(hardReader.curPose.getHeading())+90);
                     //driver.rotation.calculateGain(driver.turnToAbsolute(field.HM, hardReader.curPose), runtime.seconds());
                 }
             }


             if (shooterState!= RobotState.HIGH&&shooterState!= RobotState.POWER) {
                 shooter.safetySwitch();
             }
             else{
                  shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
             }

             if(!gamepad1.left_bumper&&shooterState== RobotState.INDEXING){
                 roller.fallOut();
                 if(lift.timeUp()) {
                     roller.upToSpeed();
                 }else{
                     shooter.liftDown();
                     roller.upToSpeed(-.8);
                 }
             }
             else if(gamepad1.right_bumper&&shooterState== RobotState.INDEXING){
                 roller.tuckIn();
                 roller.upToSpeed(-.8);
             }
             else{
                 roller.upToSpeed(0);
                }

             if (y1.wasJustPressed()) {
                 shooterState = RobotState.POWER;
                 lift.init();
             }
             if (b1.wasJustPressed()) {
                 shooterState = RobotState.HIGH;
                 lift.init();
                 // driver.turnAsync(field.HM);

             }
             if (a1.wasJustPressed()) {
                 shooterState = RobotState.WOBBLE;
             }
             if(x1.wasJustPressed()){
                 shooterState = RobotState.INDEXING;
             }
             if(shooterState !=RobotState.HIGH&&shooterState!=RobotState.POWER){
                 turning = false;
             }
             cycles++;
             double t1 = runtime.milliseconds() / cycles;

             telemetry.addData("Loop time", t1);
             telemetry.addData("shooter velo", hardReader.shooterV);
          //   telemetry.addData("Intake amps", gen.outerRollerMI.getCurrent(CurrentUnit.AMPS)+gen.outerRollerMII.getCurrent(CurrentUnit.AMPS));
          //   telemetry.addData("Shooter amps", gen.flyWheelM.getCurrent(CurrentUnit.AMPS)+gen.flyWheelM1.getCurrent(CurrentUnit.AMPS));
          //   telemetry.addData("Drive amps", gen.frontRightM.getCurrent(CurrentUnit.AMPS)+gen.frontLeftM.getCurrent(CurrentUnit.AMPS)+gen.backLeftM.getCurrent(CurrentUnit.AMPS)+gen.backRightM.getCurrent(CurrentUnit.AMPS));
             telemetry.addData("x", hardReader.curPose.getX());
             telemetry.addData("y", hardReader.curPose.getY());
             telemetry.addData("heading", hardReader.curPose.getHeading());
             telemetry.addData("State ",shooterState);
             telemetry.addData("ring velo", subs.magTrak.curVelo);
             telemetry.addData("ring pos", subs.hardReader.curX);
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


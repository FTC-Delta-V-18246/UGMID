package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class intake {
    private DcMotorEx rollerMotorII = null;
    private DcMotorEx rollerMotorI = null;
    private Servo outerRollerServoLeft, outerRollerServoRight;
    private LinearOpMode opModeObj;
    public static double O = .64  , I = .18; //outer roller's center should be about 3.25in off the ground
    public static double leftOffset = .02;
    public static double rightOffset = 0;
    public static double intakePower = 1;
    public intake(LinearOpMode opMode, hardwareGenerator gen){
        opModeObj = opMode;
        rollerMotorI = gen.outerRollerMI;
        rollerMotorII = gen.outerRollerMII;
        outerRollerServoLeft = gen.intakeServoL;
        outerRollerServoRight = gen.intakeServoR;
    }
    public void upToSpeed(){
       rollerMotorI.setPower(intakePower);
        rollerMotorII.setPower(intakePower);
    }
    public void upToSpeed(double targetVelo){
       rollerMotorI.setPower(targetVelo);
        rollerMotorII.setPower(targetVelo);

    }
    public void fallOut(){
        outerRollerServoLeft.setPosition(O+leftOffset);
        outerRollerServoRight.setPosition(O+rightOffset);
    }
    public void tuckIn(){
        outerRollerServoLeft.setPosition(I+leftOffset);
        outerRollerServoRight.setPosition(I+rightOffset);
    }
    public void moveIt(){
        outerRollerServoLeft.setPosition(.9+leftOffset);
        outerRollerServoRight.setPosition(.9+rightOffset);
    }
    public void toPosition(double pos){
        outerRollerServoLeft.setPosition(pos+leftOffset);
        outerRollerServoRight.setPosition(pos+rightOffset);
    }

}

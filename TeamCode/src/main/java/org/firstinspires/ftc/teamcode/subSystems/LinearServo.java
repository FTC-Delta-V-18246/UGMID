package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class LinearServo {
    private Servo pivotLeft, pivotRight;
    public LinearServo(hardwareGenerator gen){
        pivotLeft = gen.pivotServoL;
        pivotRight = gen.pivotServoR;
    }
    public void toPosition(double targetPosition){
        targetPosition = Range.clip(targetPosition,.05,.9);
        pivotLeft.setPosition(targetPosition);
        pivotRight.setPosition(targetPosition);
    }
}

 package org.firstinspires.ftc.teamcode.subSystems.retired;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.geometry.Point;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

import java.util.ArrayList;

import static java.lang.Math.abs;

 @Config
 public class PathGenerator {
     private ArrayList<Point> skeletonPath;
     public ArrayList<Point> filledPath;
     private ArrayList<Double> targetVelo;
     private ArrayList<Double> skeletonHeadingPath;
     public ArrayList<Double> filledHeadingPath;

     public Point lookAheadPoint;

     public int wayPointIndex = 0;

     public static double maxPathVelo = 1.52;
     public static double maxPathAccel = .762;
     public static double kConstant = 2;
     public static double lookAheadRadius = 7;
     public static double trackWidth= 17;
     public static double kP = .01;
     public static double kV = 1.0/(maxPathVelo);
     public static double kA = .002;
     private boolean generatePathBol = false;

     private odometry missileObj;
     private LinearOpMode opModeObj;
     private drive driveObj;

     private ElapsedTime accelTime = new ElapsedTime();

     private double prevLeftVelocity = 0;
     private double prevRightVelocity = 0;

     public PathGenerator(LinearOpMode opMode, odometry missile, drive vroom) {
         skeletonPath = new ArrayList<Point>();
         filledPath = new ArrayList<Point>();
         skeletonHeadingPath = new ArrayList<Double>();
         filledHeadingPath = new ArrayList<Double>();
         targetVelo = new ArrayList<Double>();
         missileObj = missile;
         lookAheadPoint = missileObj.robotLocation;
         opModeObj = opMode;
         driveObj = vroom;
     }

     /**
      * @param newPoint For the point that we are adding to the path. Points are reached in order of entry
      */
     public void addPoint(Point newPoint, double heading) {
         skeletonPath.add(newPoint);
         skeletonHeadingPath.add(heading);
     }

     public void addPoint(double x, double y, double heading) {
         Point newPoint = new Point(x,y);
         skeletonPath.add(newPoint);
         skeletonHeadingPath.add(heading);

     }

     public boolean generatePath() {
         if(!generatePathBol){
             filler(.3);
             generatePathBol = true;
             return generatePathBol;
         }
         return true;
         //targetVeloAtI(filledPath);
     }
     public boolean runPath(){
         generatePath();
         if(missileObj.robotLocation.minus(filledPath.get(filledPath.size()-1)).magnitude()<1){
             driveObj.driveT(0,0);
             return true;
         }
         else{
             findLookaheadPoint(filledPath);
             //MotorPowerController(missileObj.leftWheelVelo, missileObj.rightWheelVelo, signedCurvatureLookahead(), smoothedPath);
             //driveObj.moveToPoint(lookAheadPoint, filledHeadingPath.get(wayPointIndex+1));
         }
         opModeObj.telemetry.addData("lookahead point", lookAheadPoint);
         opModeObj.telemetry.addData("goal point", filledPath.get(wayPointIndex+1));
         opModeObj.telemetry.addData("x", missileObj.globalPositionX);
         opModeObj.telemetry.addData("y", missileObj.globalPositionY);
         opModeObj.telemetry.addData("heading", missileObj.globalPositionTheta);
         opModeObj.telemetry.update();
         return false;
     }

     public void filler(double spacing) {
         int originalSize = skeletonPath.size();
         for (int i = 0; i < originalSize - 1; i++) { //originally -2
             //translational interpolation
             Vector2d tempVector = new Vector2d(skeletonPath.get(i + 1).x - skeletonPath.get(i).x,skeletonPath.get(i + 1).y - skeletonPath.get(i).y);
             int pointsThatFit = (int) Math.ceil(tempVector.magnitude() / spacing);
             tempVector = tempVector.div(tempVector.magnitude()).times(spacing);

             for (int n = 0; n < pointsThatFit; n++) {
                 filledPath.add(skeletonPath.get(i).plus(tempVector.times(n)));
                 //tempVector = tempVector.div(n);
             }
             //heading interpolation
             double holdHeading = 0;
             if(skeletonHeadingPath.get(i+1)==null){
                 for(int j = i; 0<=j; j--){
                     if(skeletonHeadingPath.get(j)!=null){
                         holdHeading = skeletonHeadingPath.get(j);
                         break;
                     }
                 }

                 for(int j = i;  j<i+pointsThatFit+1; j++){
                     filledHeadingPath.add(holdHeading);
                 }
             }
             else if(pointsThatFit!=0) {
                 double headingDifference = smallestAngleError(skeletonHeadingPath.get(i+1),skeletonHeadingPath.get(i))/pointsThatFit;
                 for(int j = i;  j<i+pointsThatFit+1; j++){
                     double interpolatedHeadingatJ = holdHeading+j*headingDifference;
                     if(interpolatedHeadingatJ<0||interpolatedHeadingatJ>360)
                         interpolatedHeadingatJ%=360;
                     filledHeadingPath.add(interpolatedHeadingatJ);
                 }
             }


         }
         filledPath.add(skeletonPath.get(skeletonPath.size() - 1));
         filledHeadingPath.add(skeletonHeadingPath.get(skeletonHeadingPath.size() - 1));
     }

     public void reverseList() {
         ArrayList<Point> tempRevList = new ArrayList<Point>();
         for (int i = (skeletonPath.size() - 1); i > -1; i--) {
             tempRevList.add(skeletonPath.get(i));
         }
         for (int i = 0; i < tempRevList.size() - 1; i++) {
             skeletonPath.set(i, tempRevList.get(i));
         }
     }

     public ArrayList<Double> reverseList(ArrayList<Double> origList) {
         ArrayList<Double> tempRevList = new ArrayList<Double>();
         for (int i = (origList.size() - 1); i > -1; i--) {
             tempRevList.add(origList.get(i));
         }
         for (int i = 0; i < tempRevList.size() - 1; i++) {
             origList.set(i, tempRevList.get(i));
         }
         return origList;
     }

     private double distanceBetweenPoints(ArrayList<Point> givenArray, int pointIndex) {
         double totalDistance = 0;
         for (int i = 1; i < pointIndex + 1; i++) {
             totalDistance += Math.sqrt(
                     Math.pow(givenArray.get(i).x - givenArray.get(i - 1).x, 2)
                             + Math.pow(givenArray.get(i).y - givenArray.get(i - 1).y, 2));
         }
         return totalDistance;
     }

     private double curvatureBetweenPoints(Point one, Point two, Point three) {
         if (one.x == two.x) {
             one.x += .00001;
         }
         double k1 = 0.5 * (Math.pow(one.x, 2) + Math.pow(one.y, 2) - Math.pow(two.x, 2) - Math.pow(two.y, 2) / (one.x - two.x));
         double k2 = (one.y - two.y) / (one.x - two.x);
         double b = .5 * (Math.pow(two.x, 2) - 2 * two.x * k1 + Math.pow(two.y, 2) - Math.pow(three.x, 2) + 2 * three.x * k1 - Math.pow(three.y, 2)) /
                 (three.x * k2 - three.y + two.y - two.x * k2);
         double a = k1 - k2 * b;

         double r = Math.sqrt(Math.pow(one.x - a, 2) + (Math.pow(one.y - b, 2)));
         double curvature = 1 / r;
         if (Double.isNaN(curvature)) {
             return 0;
         }
         return curvature;

     }

     private void targetVeloAtI(ArrayList<Point> givenArray) {
         if (givenArray.size() < 3) {
             //Insert error handling code here
         }
         double distance;
         //double distance = distanceFormula(givenArray.get(givenArray.size()-1),givenArray.get(givenArray.size()-2));
         double oldTargetVelo = Math.min(maxPathVelo, kConstant / .0001);
         double newTargetVelo;
         targetVelo.add(oldTargetVelo);
         for (int i = 1; i < givenArray.size() - 2; i--) {
             distance = distanceFormula(givenArray.get(givenArray.size() - i), givenArray.get(givenArray.size() - i - 1));
             oldTargetVelo = Math.min(maxPathVelo, kConstant / curvatureBetweenPoints(givenArray.get(givenArray.size() - i), givenArray.get(givenArray.size() - i - 1), givenArray.get(givenArray.size() - i - 2)));
             newTargetVelo = Math.sqrt(Math.pow(targetVelo.get(i - 1), 2) + 2 * maxPathAccel * distance); //add rate limiter?
             targetVelo.add(Math.min(oldTargetVelo, newTargetVelo));
         }
     }

     private Point findLookaheadPoint(ArrayList<Point> givenArray) {
         Point startingPoint, endingPoint;

         Point centerLocation = missileObj.robotLocation;
         double a, b, c, discriminant, lookAheadDistance;
         double t1, t2;
         Vector2d d, f;
         for (int i = wayPointIndex; i < givenArray.size() - 2; i++) {
             startingPoint = givenArray.get(i);
             endingPoint = givenArray.get(i + 1);
             lookAheadDistance = lookAheadRadius;
             d = new Vector2d(endingPoint.x - startingPoint.x, endingPoint.y - startingPoint.y);
             f = new Vector2d(startingPoint.x - centerLocation.x, startingPoint.y - startingPoint.y);
             a = d.dot(d);
             b = 2 * f.dot(d);
             c = f.dot(f) - lookAheadDistance * lookAheadDistance;
             discriminant = b * b - 4 * a * c;
             if (discriminant < 0) {

             } else {
                 discriminant = Math.sqrt(discriminant);
                 t1 = (-b - discriminant) / (2 * a);
                 t2 = (-b + discriminant) / (2 * a);
                 if (t1 >= 0 && t1 <= 1) {
                     lookAheadPoint = startingPoint.plus(d.times(t1));
                     wayPointIndex = i;
                     return lookAheadPoint;
                 }
                 if (t2 >= 0 && t2 <= 1) {
                     lookAheadPoint =  startingPoint.plus(d.times(t2));
                     wayPointIndex = i;
                     return lookAheadPoint;
                 }
                 //otherwise, no intersection
                 opModeObj.telemetry.addData("no lookahead", ":(");
             }
         }
         return lookAheadPoint;
     }

     private double curvatureLookahead(){
         double a = -Math.tan(missileObj.globalPositionTheta);
         double b = 1;
         double c = Math.tan(missileObj.globalPositionTheta) * missileObj.globalPositionX-missileObj.globalPositionY;
         double x = Math.abs(a*lookAheadPoint.x+b*lookAheadPoint.y+c)/ Math.sqrt(a*a+b*b);
         return 2*x/(lookAheadRadius*lookAheadRadius);
     }

     private int findClosestPoint(ArrayList<Point> givenArray){
         int closestPointIndex = 0;
         double closestDistance = 0;
         double curDistance;
         Point centerLocation = missileObj.robotLocation;
         for(int i = 0; i < givenArray.size()-1; i++){
             curDistance = distanceFormula(centerLocation, givenArray.get(i));
             if(curDistance>closestDistance){
                 closestPointIndex = i;
                 closestDistance = curDistance;
             }
         }
         return closestPointIndex;
     }

     private double signedCurvatureLookahead(){
         double robotAngle = missileObj.globalPositionTheta;
         double side = Math.signum(Math.sin(robotAngle)*(lookAheadPoint.x-missileObj.globalPositionY)
                 - Math.cos(robotAngle)*(lookAheadPoint.y-missileObj.globalPositionX));
         return side*curvatureLookahead();
     }

     private double distanceFormula(Point one, Point two){
         return Math.sqrt(Math.pow(two.x-one.x,2)+ Math.pow(two.y-one.y,2));
     }

     private void MotorPowerController(double curLeftVelocity, double curRightVelocity, double signedCurvature, ArrayList<Point> givenArray){
         double timeDif = accelTime.seconds();
         double leftTargetVelocity = targetVelo.get(wayPointIndex)*(2+signedCurvature*trackWidth)/2;
         double rightTargetVelocity = targetVelo.get(wayPointIndex)*(2-signedCurvature*trackWidth)/2;
         double leftTargetAccel = (leftTargetVelocity-prevLeftVelocity)/timeDif;
         double rightTargetAccel = (rightTargetVelocity - prevRightVelocity)/timeDif;
         driveObj.driveT(FFFB(leftTargetVelocity,leftTargetAccel,curLeftVelocity),FFFB(rightTargetVelocity,rightTargetAccel,curRightVelocity));
         accelTime.reset();
     }
     private double FFFB(double targetVelo, double targetAccel, double curVelo){
         double gain = kV*targetVelo+kA*targetAccel+kP*(targetVelo-curVelo);
         return gain;
     }

     private double smallestAngleError(double goalAngle, double curAngle){
         double error;
         double wrappingError = -360 - curAngle + goalAngle;
         double straightError = curAngle - goalAngle;
         if (abs(wrappingError) < abs(straightError)) {
             error = -360 - curAngle + goalAngle;
             error *= -1;
         } else {
             error = curAngle - goalAngle;
         }
         return error;
     }

 }

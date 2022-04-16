package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {
    SampleMecanumDrive drive;
    int cameraMonitorViewId; 
    public OpenCvCamera webcam;
    OpMode opMode;
    public DetectionPipeline detectionPipeline;
    public DcMotorEx slider; 
    public DcMotorEx intake; 
    public DcMotor carusel;
    public Servo cuva;
    public DcMotorEx rotire;
    public ColorSensor color;
    private boolean bCameraOpen;
  
    public Robot(OpMode op)
    {
      opMode = op;
		  hardwareMap = opMode.hardwareMap;
      
      SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
      cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

  
      DcMotorEx slider = hardwareMap.get(DcMotorEx.class, "slider");
      DcMotorEx intake =  hardwareMap.get(DcMotorEx.class, "intake");
      DcMotor carusel = hardwareMap.get(DcMotor.class, "carusel");
      Servo cuva = hardwareMap.get(Servo.class,"cuva");
      DcMotorEx rotire = hardwareMap.get(DcMotorEx.class,"rotire");
      ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
      bCameraOpen = false;
    }
    public startCamera()
    {
       webcam.setPipeline(detectionPipeline);
      
       webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpen = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public isCameraOpened() { return bCameraOpen };
  
 
  
}

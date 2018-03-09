package org.usfirst.frc.team2577.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

public class MecanumDrive {
	private Talon frontLeft;
	private Talon frontRight;
	private Talon rearLeft;
	private Talon rearRight;
	private ADXRS450_Gyro gyro;
	private double lastAngle = 0;
	private double averageVelocity = 0;
	private Timer t;
	
	public MecanumDrive(int frontLeftPort, int frontRightPort,  int rearLeftPort, int rearRightPort){
		frontLeft = new Talon(frontLeftPort);
		frontRight = new Talon(frontRightPort);
		rearLeft = new Talon(rearLeftPort);
		rearRight = new Talon(rearRightPort);
		
		gyro = new ADXRS450_Gyro();
		
		t = new Timer();
		t.start();
		
		gyro.calibrate();
	}
	
	public void driveForward(double speed){
		drive(0, 0, speed, true);
	}
	
	/**
	 * 
	 * @param angle     angle of strafe (0 is forward, Pi/2 is right, Pi is back)
	 * @param rotation  rotation of robot (from -1 to 1)
	 * @param magnitude    magnitude of strafe
	 */
	public void drive(double direction, double rotation, double magnitude, boolean applyAdjustment){
		boolean relativeToSelf = true;
		double gyroAngle = (gyro.getAngle()); //Get the current angle of the gyro in radians (Always positive)
		if(relativeToSelf){
			
			double dAngle = (lastAngle - gyroAngle);
			double dTime = t.get();
			t.reset();
			double velocity = dAngle/dTime;
			
			averageVelocity = (averageVelocity * 3 + velocity)/4;
			
			lastAngle = gyroAngle;
			System.out.println("Rotation Goal: "+rotation*300);
			System.out.println("Average Velocity: "+averageVelocity);
			System.out.println("Offset: " +(rotation*300 - averageVelocity));
			
			if(applyAdjustment){
				rotation += (rotation - averageVelocity/300.0)*1.5;
			}
			
			
			direction += Math.PI/4.0;  //Strafe direction needs to be offset slightly
			double v1 = magnitude * Math.cos(direction) + rotation;
	        double v2 = magnitude * Math.sin(direction) - rotation;
	        double v3 = magnitude * Math.sin(direction) + rotation;
	        double v4 = magnitude * Math.cos(direction) - rotation;
	        frontLeft.set(v1);
			frontRight.set(v2);
			rearLeft.set(v3);
			rearRight.set(v4);
		}
	}
}

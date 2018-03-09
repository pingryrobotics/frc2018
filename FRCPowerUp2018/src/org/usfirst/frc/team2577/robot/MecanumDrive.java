package org.usfirst.frc.team2577.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

public class MecanumDrive {
	private TalonSRX frontLeft;
	private TalonSRX frontRight;
	private TalonSRX rearLeft;
	private TalonSRX rearRight;
	private ADXRS450_Gyro gyro;
	private double lastAngle = 0;
	private double averageVelocity = 0;
	private double averageGoal = 0;
	private Timer t;
	
	public MecanumDrive(int frontLeftPort, int frontRightPort,  int rearLeftPort, int rearRightPort){
		frontLeft = new TalonSRX(frontLeftPort);
		frontRight = new TalonSRX(frontRightPort);
		rearLeft = new TalonSRX(rearLeftPort);
		rearRight = new TalonSRX(rearRightPort);
		
		gyro = new ADXRS450_Gyro();
		
		t = new Timer();
		t.start();
		
		frontLeft.setInverted(true);
		rearLeft.setInverted(true);
		gyro.calibrate();
	}
	
	public void driveForward(double speed){
		drive(0, 0, speed);
	}
	
	/**
	 * 
	 * @param angle     angle of strafe (0 is forward, Pi/2 is right, Pi is back)
	 * @param rotation  rotation of robot (from -1 to 1)
	 * @param magnitude    magnitude of strafe
	 */
	public void drive(double direction, double rotation, double magnitude){
		double gyroAngle = (gyro.getAngle()); //Get the current angle of the gyro in radians (Always positive)
		double dAngle = (lastAngle - gyroAngle);
		double dTime = t.get();
		t.reset();
		double velocity = dAngle/dTime;
		
		averageGoal = (averageGoal * 3 + rotation)/4.0;
		
		averageVelocity = (averageVelocity * 3 + velocity)/4;
		
		lastAngle = gyroAngle;
		
		rotation += (averageGoal - averageVelocity/300.0)*1.5;
		
		
		direction += Math.PI/4.0;  //Strafe direction needs to be offset slightly
		double v1 = magnitude * Math.cos(direction) + rotation;
        double v2 = magnitude * Math.sin(direction) - rotation;
        double v3 = magnitude * Math.sin(direction) + rotation;
        double v4 = magnitude * Math.cos(direction) - rotation;
        frontLeft.set(ControlMode.PercentOutput, v1);
		frontRight.set(ControlMode.PercentOutput, v2);
		rearLeft.set(ControlMode.PercentOutput, v3);
		rearRight.set(ControlMode.PercentOutput, v4);
	}
}

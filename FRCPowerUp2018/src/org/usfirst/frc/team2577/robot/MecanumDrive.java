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
	 * Mecanum drive code.
	 * Uses gyro values to compensate for turn drifting.
	 * Polar drive system.
	 * 
	 * @param angle     angle of strafe (0 is forward, Pi/2 is right, Pi is back)
	 * @param rotation  rotation of robot (from -1 to 1)
	 * @param magnitude    magnitude of strafe
	 */
	public void drive(double direction, double rotation, double magnitude){
		//Get the current gyro angle
		double gyroAngle = gyro.getAngle();
		//Find the change since our last angle
		double dAngle = (lastAngle - gyroAngle);
		//Find the change in time since out last measurement
		double dTime = t.get();
		t.reset();
		//Find the change in angle over time (angular velocity)
		double velocity = dAngle/dTime;
		
		//Find the AVERAGE velocity (just a smoothed out velocity that has been averaged to minimize static noise and improve accuracy)
		averageVelocity = (averageVelocity * 3 + velocity)/4;
		
		//Find the AVERAGE rotational goal (smoothed out)
		averageGoal = (averageGoal * 3 + rotation)/4.0;
		
		//Update the last angle
		lastAngle = gyroAngle;
		
		//Update the rotational goal to compensate for how off we are from the goal.
		//Dividing by 300 to convert the degrees per second into power for a motor. We found that about 300 degrees per second is a 1 in turning power.
		//The 1.5x is a multiplier to make sure the offset is applied enough to have an actual effect.
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

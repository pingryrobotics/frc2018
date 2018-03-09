package org.usfirst.frc.team2577.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Hand {
	private TalonSRX rightFinger;
	private TalonSRX leftFinger;
	
	public Hand(int rightPort, int leftPort){
		rightFinger = new TalonSRX(rightPort);
		leftFinger = new TalonSRX(leftPort);
	}
	
	public void suckAtPower(double power){
		leftFinger.set(ControlMode.PercentOutput, -power);
		rightFinger.set(ControlMode.PercentOutput, power);
	}
	
	public void suckIn(){
		leftFinger.set(ControlMode.PercentOutput, -1);
		rightFinger.set(ControlMode.PercentOutput, 1);
	}
	
	public void spitAtPower(double power){
		leftFinger.set(ControlMode.PercentOutput, power);
		rightFinger.set(ControlMode.PercentOutput, -power);
	}
	
	public void spitOut(){
		leftFinger.set(ControlMode.PercentOutput, 1);
		rightFinger.set(ControlMode.PercentOutput, -1);
	}
	
	public void stop(){
		leftFinger.set(ControlMode.PercentOutput, 0);
		rightFinger.set(ControlMode.PercentOutput, 0);
	}
}

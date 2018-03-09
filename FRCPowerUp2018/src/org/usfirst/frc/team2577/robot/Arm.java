package org.usfirst.frc.team2577.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Arm {
	private TalonSRX shoulder;
	private TalonSRX wrist;
	private TalonSRX winch;
	
	//All wrist and shoulder positions are set in the resetWristValues and resetShoulderValues functions
	public int wristDownPosition;
	public int shoulderDownPosition;
	
	public int wristUpPosition;
	public int wristSwitchPosition;
	public int wristForwardPosition;
	public int shoulderSwitchPosition;
	public int shoulderForwardPosition;
	public int shoulderUpPosition;
	
	public int shoulderWristFoldBottomLimit;
	public int shoulderWristFoldTopLimit;
	
	
	
	/**
	 * Arm controls
	 * @param shoulderPort CAN device ID for the shoulder
	 * @param wristPort CAN device ID for the wrist
	 * @param winchPort CAN device ID for the winch
	 */
	public Arm(int shoulderPort, int wristPort, int winchPort){
		shoulder = new TalonSRX(shoulderPort);
		wrist = new TalonSRX(wristPort);
		winch = new TalonSRX(winchPort);
		resetWristValues(3900);
		resetShoulderValues(2850);
	}
	
	public void resetWristValues(int newWristDown){
		wristDownPosition = newWristDown;
		wristUpPosition = wristDownPosition + 1100;
		wristSwitchPosition = wristDownPosition + 500;
		wristForwardPosition = wristDownPosition - 500;
	}
	
	public void resetShoulderValues(int newShoulderDown){
		shoulderDownPosition = newShoulderDown;
		shoulderSwitchPosition = shoulderDownPosition - 1200;
		shoulderForwardPosition = shoulderDownPosition - 5800;
		shoulderUpPosition = shoulderDownPosition - 8650;
		
		shoulderWristFoldBottomLimit = shoulderDownPosition - 900;
		shoulderWristFoldTopLimit = shoulderDownPosition - 8200;
		
	}
	
	public void moveShoulder(double power){
		shoulder.set(ControlMode.PercentOutput, power);
	}
	
	public int getShoulderPosition(){
		return shoulder.getSensorCollection().getPulseWidthPosition();
	}
	
	public void moveWrist(double power){
		wrist.set(ControlMode.PercentOutput, power);
	}
	
	public int getWristPosition(){
		return wrist.getSensorCollection().getPulseWidthPosition();
	}
	
	public void moveWinch(double power){
		winch.set(ControlMode.PercentOutput, power);
	}
	
	public void moveShoulderTowardsTarget(int target){
		int armOffset = target - getShoulderPosition(); //Negative means we need to come up
		double armPower = Math.min(1.0, Math.max(-1.0, Math.pow(armOffset/500.0, 3)));
		moveShoulder(-armPower); //Positive power is up
	}
	
	public void moveWristTowardsTarget(int target){
		int wristOffset = target - getWristPosition(); //If is greater than 0, move wrist up
		double wristPower = 0.8*Math.min(1.0, Math.max(-1.0, Math.pow(wristOffset/100.0, 3)));
		moveWrist(wristPower); // positive power is up
	}
}

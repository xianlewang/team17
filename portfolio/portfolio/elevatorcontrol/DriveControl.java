package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.HoistwayLimitSensorCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.EmergencyBrakePayload.ReadableEmergencyBrakePayload;
import simulator.payloads.HoistwayLimitPayload.ReadableHoistwayLimitPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * DriveControl uses messages from mEmergencyBrake, mHoistwayLimit, mAtFloor, m
 * mCarWeight, mLevel, mDoorClosed, and mDesiredFloor to determine how to drive the car
 *  
 *
 * @author jiangh
 */

public class DriveControl extends Controller {
	//local physical state
	private WriteableDrivePayload localDrive;
	private ReadableDriveSpeedPayload localDriveSpeed; 
    
	//Network Interface
	//mEmergencyBrake
	private ReadableCanMailbox networkEmergencyBrake;
	private BooleanCanPayloadTranslator mEmergencyBrake;
	
	//mHoistwayLimit
	private ReadableCanMailbox networkHoistwayLimitUp;
	private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitUp;
	private ReadableCanMailbox networkHoistwayLimitDown;
	private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitDown;
	
	//mAtFloor	
	private ReadableCanMailbox networkAtFloorOneFront;
	private ReadableCanMailbox networkAtFloorOneBack;
	private ReadableCanMailbox networkAtFloorTwoBack;
	private ReadableCanMailbox networkAtFloorThreeFront;
	private ReadableCanMailbox networkAtFloorFourFront;
	private ReadableCanMailbox networkAtFloorFiveFront;
	private ReadableCanMailbox networkAtFloorSixFront;
	private ReadableCanMailbox networkAtFloorSevenFront;
	private ReadableCanMailbox networkAtFloorSevenBack;
	private ReadableCanMailbox networkAtFloorEightFront;
	private AtFloorCanPayloadTranslator mAtFloorOneFront;
	private AtFloorCanPayloadTranslator mAtFloorOneBack;
	private AtFloorCanPayloadTranslator mAtFloorTwoBack;
	private AtFloorCanPayloadTranslator mAtFloorThreeFront;
	private AtFloorCanPayloadTranslator mAtFloorFourFront;
	private AtFloorCanPayloadTranslator mAtFloorFiveFront;
	private AtFloorCanPayloadTranslator mAtFloorSixFront;
	private AtFloorCanPayloadTranslator mAtFloorSevenFront;
	private AtFloorCanPayloadTranslator mAtFloorSevenBack;
	private AtFloorCanPayloadTranslator mAtFloorEightFront;
	
	//mLevel
	private ReadableCanMailbox networkLevelUp;
	private LevelingCanPayloadTranslator mLevelUp;
	private ReadableCanMailbox networkLevelDown;
	private LevelingCanPayloadTranslator mLevelDown;
	
	//mDoorClosed
	private ReadableCanMailbox networkDoorClosedFrontLeft;  
	private DoorClosedCanPayloadTranslator mDoorClosedFrontLeft; 
	private ReadableCanMailbox networkDoorClosedFrontRight;  
	private DoorClosedCanPayloadTranslator mDoorClosedFrontRight; 
	private ReadableCanMailbox networkDoorClosedBackLeft;  
	private DoorClosedCanPayloadTranslator mDoorClosedBackLeft; 
	private ReadableCanMailbox networkDoorClosedBackRight;  
	private DoorClosedCanPayloadTranslator mDoorClosedBackRight; 
	
	//mDoorMotor
	private ReadableCanMailbox networkDoorMotorFrontLeft; 
	private DoorMotorCanPayloadTranslator mDoorMotorFrontLeft;
	private ReadableCanMailbox networkDoorMotorFrontRight; 
	private DoorMotorCanPayloadTranslator mDoorMotorFrontRight;
	private ReadableCanMailbox networkDoorMotorBackLeft; 
	private DoorMotorCanPayloadTranslator mDoorMotorBackLeft;
	private ReadableCanMailbox networkDoorMotorBackRight; 
	private DoorMotorCanPayloadTranslator mDoorMotorBackRight;
	
	//mDesiredFloor
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	
	//mDrive
	private WriteableCanMailbox networkDrive;
	private DriveCommandCanPayloadTranslator mDrive;
	
	//mDriveSpeed
	private WriteableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;
	
	// mCarWeight
	// private ReadableCanMailbox networkCarWeight;
	// private CarWeightCanPayloadTranslator mCarWeight;
	
	//additional internal state variables
	private Speed commandSpeed = Speed.STOP; 
	private Direction desiredDirection = Direction.STOP; 
	private int currentFloor;
	
	private int desiredFloor;
	
	//store the period for the controller
	private SimTime period;
	
	// enumerate states
	private enum State {
	  STOP,
	  LEVEL_UP,
	  LEVEL_DOWN,
	  SLOW_UP,
	  SLOW_DOWN,
	}
	
	//state variable initialized to the initial state STOP
	private State state = State.STOP;
	
	/**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
	public DriveControl(SimTime period, boolean verbose) {
		super("DriveControl", verbose);
		
		this.period = period;
		
		log("Created DriveControl with period = ", period);
		
		//initialize physical state
		localDrive = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localDrive, period);
		
		localDriveSpeed = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(localDriveSpeed);
		
		//initialize network interface
		/*****************************registrate network message*****************************/
		//mEmergencyBrake
		networkEmergencyBrake = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mEmergencyBrake = new BooleanCanPayloadTranslator(networkEmergencyBrake);
		canInterface.registerTimeTriggered(networkEmergencyBrake);
		
		//mHoistwayLimit
		networkHoistwayLimitUp = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		mHoistwayLimitUp = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimitUp, Direction.UP);
		canInterface.registerTimeTriggered(networkHoistwayLimitUp);
		
		networkHoistwayLimitDown = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		mHoistwayLimitDown = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimitDown, Direction.DOWN);
		canInterface.registerTimeTriggered(networkHoistwayLimitDown);
		
		//mAtFloor
		//1 front
		networkAtFloorOneFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(1, Hallway.FRONT));
		mAtFloorOneFront = new AtFloorCanPayloadTranslator(networkAtFloorOneFront, 1, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorOneFront);
		//1 back
		networkAtFloorOneBack = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(1, Hallway.BACK));
		mAtFloorOneBack = new AtFloorCanPayloadTranslator(networkAtFloorOneBack, 1, Hallway.BACK);
		canInterface.registerTimeTriggered(networkAtFloorOneBack);
		//2 back        
		networkAtFloorTwoBack = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(2, Hallway.BACK));
		mAtFloorTwoBack = new AtFloorCanPayloadTranslator(networkAtFloorTwoBack, 2, Hallway.BACK);
		canInterface.registerTimeTriggered(networkAtFloorTwoBack);
		//3 front
		networkAtFloorThreeFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(3, Hallway.FRONT));
		mAtFloorThreeFront = new AtFloorCanPayloadTranslator(networkAtFloorThreeFront, 3, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorThreeFront);
		//4 front
		networkAtFloorFourFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(4, Hallway.FRONT));
		mAtFloorFourFront = new AtFloorCanPayloadTranslator(networkAtFloorFourFront, 4, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorFourFront);
		//5 front        
		networkAtFloorFiveFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(5, Hallway.FRONT));
		mAtFloorFiveFront = new AtFloorCanPayloadTranslator(networkAtFloorFiveFront, 5, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorFiveFront);
		//6 front
		networkAtFloorSixFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(6, Hallway.FRONT));
		mAtFloorSixFront = new AtFloorCanPayloadTranslator(networkAtFloorSixFront, 6, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorSixFront);
		//7 front        
		networkAtFloorSevenFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(7, Hallway.FRONT));
		mAtFloorSevenFront = new AtFloorCanPayloadTranslator(networkAtFloorSevenFront, 7, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorSevenFront);
		//7 back   
		networkAtFloorSevenBack = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(7, Hallway.BACK));
		mAtFloorSevenBack = new AtFloorCanPayloadTranslator(networkAtFloorSevenBack, 7, Hallway.BACK);
		canInterface.registerTimeTriggered(networkAtFloorSevenBack);
		//8 front        
		networkAtFloorEightFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(8, Hallway.FRONT));
		mAtFloorEightFront = new AtFloorCanPayloadTranslator(networkAtFloorEightFront, 8, Hallway.FRONT);
		canInterface.registerTimeTriggered(networkAtFloorEightFront);
		
		//mLevel
		networkLevelUp = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		mLevelUp = new LevelingCanPayloadTranslator(networkLevelUp, Direction.UP);
		canInterface.registerTimeTriggered(networkLevelUp);
		
		networkLevelDown = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		mLevelDown = new LevelingCanPayloadTranslator(networkLevelDown, Direction.DOWN);
		canInterface.registerTimeTriggered(networkLevelDown);
		
		//mDoorClosed
		//Front Left
		networkDoorClosedFrontLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
		mDoorClosedFrontLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontLeft, Hallway.FRONT, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorClosedFrontLeft);
		//Front Right
		networkDoorClosedFrontRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
		mDoorClosedFrontRight = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontRight, Hallway.FRONT, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorClosedFrontRight);
		//Back Left
		networkDoorClosedBackLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
		mDoorClosedBackLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedBackLeft, Hallway.BACK, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorClosedBackLeft);
		//Back Right
		networkDoorClosedBackRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
		mDoorClosedBackRight = new DoorClosedCanPayloadTranslator(networkDoorClosedBackRight, Hallway.BACK, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorClosedBackRight);
		
		//mDoorMotor
		networkDoorMotorFrontLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
		mDoorMotorFrontLeft = new DoorMotorCanPayloadTranslator(networkDoorMotorFrontLeft, Hallway.FRONT, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorMotorFrontLeft);

		networkDoorMotorFrontRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
		mDoorMotorFrontRight = new DoorMotorCanPayloadTranslator(networkDoorMotorFrontRight, Hallway.FRONT, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorMotorFrontRight);

		networkDoorMotorBackLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
		mDoorMotorBackLeft = new DoorMotorCanPayloadTranslator(networkDoorMotorBackLeft, Hallway.BACK, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorMotorBackLeft);

		networkDoorMotorBackRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
		mDoorMotorBackRight = new DoorMotorCanPayloadTranslator(networkDoorMotorBackRight, Hallway.BACK, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorMotorBackRight);
		
		
		//mDesiredFloor
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		
		//mDrive
		networkDrive = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
		mDrive = new DriveCommandCanPayloadTranslator(networkDrive);
		canInterface.sendTimeTriggered(networkDrive, period);
		
		//mDriveSpeed
		networkDriveSpeed = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.sendTimeTriggered(networkDriveSpeed, period);
		
		timer.start(period);
	}
	
	/*
     * The timer callback is where the main controller code is executed.  For time
     * triggered design, this consists mainly of a switch block with a case block for
     * each state.  Each case block executes actions for that state, then executes
     * a transition to the next state if the transition conditions are met.
     */
	public void timerExpired(Object callbackData) {
		State newState = state;
		//Utility.AtFloorArray floorArray = new Utility.AtFloorArray(canInterface);
		//Utility.DoorClosedArray frontDoorClosedArray = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
		//Utility.DoorClosedArray backDoorClosedArray = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
		
		boolean allDoorClosed;
		boolean allDoorMotorStop;
		boolean levelFlag;
		
		/*******************************
		switch (state) {
			case STOP:
				 //state actions for 'STOP'
				 localDrive.set(Speed.STOP, Direction.STOP);
				 mDrive.set(Speed.STOP, Direction.STOP);
				 // mDriveSpeed.set(Speed.STOP, Direction.STOP);
				 tempCurrFloor = getCurrentFloor();
				 //#transition 'T6.1'
				 if ((tempCurrFloor != mDesiredFloor.getFloor()) && (mDoorClosed.getValue() == true)) {
					  newState = State.MOVING;
				 } 
				 else {
					  newState = state;
				 }
				 break;
			case MOVING:
				 //state actions for 'MOVING'
				 localDrive.set(Speed.SLOW, mDesiredFloor.getDirection());
				 mDrive.set(Speed.SLOW, mDesiredFloor.getDirection());
				 // mDriveSpeed.set(Speed.SLOW, movingDirection);				 				 
				 // System.out.println("mDrive.getDirection() is " + mDrive.getDirection());
				 // System.out.println("mDrive.getSpeed() is " + mDrive.getSpeed());
				 
				 boolean atFLoorFlag = false;
				 tempCurrFloor = 0;
				 for(int i = 0; i < 8; i++) {
					 atFLoorFlag = atFLoorFlag || mAtFloor[i].getValue();
					 if(atFLoorFlag) {
						 tempCurrFloor = i + 1;
						 break;
					 }					 
				 }
				 
				//#transition 'T6.2'
				 if ((atFLoorFlag == true) && (tempCurrFloor == mDesiredFloor.getFloor())) {
					  newState = State.STOP;				 
				 } 
				 else {
					  newState = state;
				 }
				 break;
			default:
				 throw new RuntimeException("State " + state + " was not recognized.");
		}
		******************************************/
		// (3)
		switch (state) {
			case STOP:
				 currentFloor = getCurrentFloor();
				 allDoorClosed = checkAllDoorClosed();	
				 allDoorMotorStop = checkAllDoorMotorStop();
				 
				 //state actions for 'STOP'
				 desiredDirection = getDesiredDirection(mDesiredFloor, currentFloor);
				 desiredFloor = getDesiredFloor();
				 
				 commandSpeed = Speed.STOP;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
				 System.out.println("currentFloor is " + currentFloor);
				 System.out.println("mDesiredFloor is " + desiredFloor);
			    System.out.println("desiredDirection is " + desiredDirection);
			    System.out.println("allDoorClosed is " + allDoorClosed);
			    System.out.println("allMotorStop is " + allDoorMotorStop);
				 
//#transition 'T6.1' STOP -> LEVEL UP
				 if ((allDoorClosed == false) && (mLevelDown.getValue() == false)) {
					  newState = State.LEVEL_UP;
				 }
//#transition 'T6.3' STOP -> SLOW UP			 
				 else if (allDoorClosed == true && allDoorMotorStop == true && (currentFloor != desiredFloor) && desiredDirection == Direction.UP) {
						newState = State.SLOW_UP;
				 }
//#transition 'T6.5' STOP -> LEVEL DOWN		 
				 else if (allDoorClosed == false && (mLevelUp.getValue() == false)) {
						newState = State.LEVEL_DOWN;
				 }
//#transition 'T6.7' STOP -> SLOW DOWN		 
				 else if (allDoorClosed && allDoorMotorStop  == true && (currentFloor != desiredFloor) && desiredDirection == Direction.DOWN) {
						newState = State.SLOW_DOWN;
				 } 
				 else {
					  newState = state;
				 }
				 break;
			case LEVEL_UP:				 			
				 //state actions for 'LEVEL UP'
				 desiredDirection = Direction.UP;
				 commandSpeed = Speed.LEVEL;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(Speed.SLOW, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 				 
//#transition 'T6.2'
				 if ((mLevelDown.getValue() == true && mLevelUp.getValue()==true) || (mEmergencyBrake.getValue()==true) || (mHoistwayLimitDown.getValue()==true) || (mHoistwayLimitUp.getValue()==true)) {
					  newState = State.STOP;				 
				 } else {
					  newState = state;
				 }
				 break;
			case LEVEL_DOWN:
				 //state actions for 'LEVEL DOWN'
				 desiredDirection = Direction.DOWN;
				 commandSpeed = Speed.LEVEL;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(Speed.SLOW, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.6'
				 if ((mLevelDown.getValue() == true && mLevelUp.getValue()==true) || (mEmergencyBrake.getValue()==true) || (mHoistwayLimitDown.getValue()==true) || (mHoistwayLimitUp.getValue()==true)) {
					  newState = State.STOP;
				 } else {
					  newState = state;
				 }
				 break;
			case SLOW_UP:
				 currentFloor = getCurrentFloor();
				 allDoorClosed = checkAllDoorClosed();	
				 allDoorMotorStop = checkAllDoorMotorStop();
				 levelFlag = mLevelDown.getValue() && mLevelUp.getValue();
				
				 //state actions for 'SLOW_UP'
				 desiredDirection = Direction.UP;
				 desiredFloor = getDesiredFloor();
				 commandSpeed = Speed.SLOW;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.4'
				 if (currentFloor == desiredFloor && levelFlag == false) {
					  newState = State.LEVEL_UP;
				 }
//#transition 'T6.9'
				 else if(mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true) {
					  newState = State.STOP;
				 }
				 else {
					  newState = state;
				 }
				 break;
			case SLOW_DOWN:
				 currentFloor = getCurrentFloor();
				 allDoorClosed = checkAllDoorClosed();	
				 allDoorMotorStop = checkAllDoorMotorStop();
				 levelFlag = mLevelDown.getValue() && mLevelUp.getValue();
				
				 //state actions for 'SLOW_DOWN'
				 desiredDirection = Direction.DOWN;
				 desiredFloor = getDesiredFloor();
				 commandSpeed = Speed.SLOW;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.8'
				 if (mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true || (currentFloor == desiredFloor && levelFlag == false)) {
					  newState = State.LEVEL_DOWN;
				 }
//#transition 'T6.10'
				 else if(mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true) {
					  newState = State.STOP;
				 }				 
				 else {
					  newState = state;
				 }
				 break;
			default:
				 throw new RuntimeException("State " + state + " was not recognized.");
		}
		
		/////////////////////////////////////////////////////////////
		
	  //log the results of this iteration
	  if (state == newState) {
			log("remains in state: ",state);
	  } else {
			log("Transition:",state,"->",newState);
	  }

	  //update the state variable
	  state = newState;

	  //report the current state
	  setState(STATE_KEY,newState.toString());

	  //schedule the next iteration of the controller
	  //you must do this at the end of the timer callback in order to restart
	  //the timer
	  timer.start(period);
	}
	
	// /*
	private int getCurrentFloor() {
		int cf = -1;
		
		if(mAtFloorOneFront.getValue()==true || mAtFloorOneBack.getValue()==true)	cf=1;
		if(mAtFloorTwoBack.getValue()==true)	cf=2;
		if(mAtFloorThreeFront.getValue()==true)	cf=3;
		if(mAtFloorFourFront.getValue()==true)	cf=4;
		if(mAtFloorFiveFront.getValue()==true)	cf=5;
		if(mAtFloorSixFront.getValue()==true)	cf=6;
		if(mAtFloorSevenFront.getValue()==true || mAtFloorSevenBack.getValue()==true)	cf=7;
		if(mAtFloorEightFront.getValue()==true)	cf=8;

		return cf;
	}
	// */
	
	private Direction getDesiredDirection(DesiredFloorCanPayloadTranslator mDesiredFloor, int currentFloor) {
		Direction d;
		if(mDesiredFloor.getFloor() > currentFloor) {
			d = Direction.UP;			
		}
		else if(mDesiredFloor.getFloor() < currentFloor) {
			d = Direction.DOWN;			
		}
		else {
			d = Direction.STOP;			
		}			
		
		return d;
	}
	
	private int getDesiredFloor() {
		int df;
		
		if(mDesiredFloor.getFloor() == 0) {
			df = 1;			
		}
		else {
			df = mDesiredFloor.getFloor();		
		}			
		
		return df;
	}
	
	
	private boolean checkAllDoorMotorStop() {
		return (mDoorMotorFrontLeft.getDoorCommand() == DoorCommand.STOP) && (mDoorMotorFrontRight.getDoorCommand()==DoorCommand.STOP) && (mDoorMotorBackLeft.getDoorCommand()==DoorCommand.STOP) && (mDoorMotorBackRight.getDoorCommand()==DoorCommand.STOP);
	}
	
	private boolean checkAllDoorClosed() {
		return mDoorClosedFrontLeft.getValue() && mDoorClosedFrontRight.getValue() &&
                	mDoorClosedBackLeft.getValue() && mDoorClosedBackRight.getValue();
	}
}

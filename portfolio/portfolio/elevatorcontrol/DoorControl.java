/*
 18649 Fall 2013
 Group 17
 Qiang Zhang(qiangz)
 Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.*;
import simulator.elevatormodules.*;
import simulator.payloads.*;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.CallArray;
import simulator.elevatorcontrol.DoorMotorCanPayloadTranslator;

/*
 * This doorControl uses
 * <mAtFloor>, <mDesiredFloor>, <mDriveSpeed>,
 * <mDoorOpened>, <mDesiredDwell>, <mCarWeight>
 */
public class DoorControl extends Controller {
    
    // define state variables
    public static enum State {
        CLOSED,
        BEFORE_OPEN,
        OPENED,
        CLOSING,
        REVERSE_OPEN,
        REVERSE_OPENED,
        REVERSE_CLOSING,
        WAIT_DISPATCHER,
    }
    // (in ms) how often this module should send out messages
    private final SimTime period;
    private final Hallway hallway;
    private final Side side;
    // physical interface
    private WriteableDoorMotorPayload door_motor;
    // input network
    private AtFloorArray mAtFloor_array;
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private ReadableCanMailbox networkDriveSpeed;
    private DriveSpeedCanPayloadTranslator mDriveSpeed;
    private ReadableCanMailbox networkDoorOpened_left;
    private ReadableCanMailbox networkDoorOpened_right;
    private DoorOpenedCanPayloadTranslator mDoorOpened;
    private DoorOpenedCanPayloadTranslator mDoorOpened_left;
    private DoorOpenedCanPayloadTranslator mDoorOpened_right;
    private DoorOpenedCanPayloadTranslator mOtherSideOpened;
    private ReadableCanMailbox networkDoorClosed_left;
    private ReadableCanMailbox networkDoorClosed_right;
    private DoorClosedCanPayloadTranslator mDoorClosed;
    private DoorClosedCanPayloadTranslator mDoorClosed_left;
    private DoorClosedCanPayloadTranslator mDoorClosed_right;
    private DoorClosedCanPayloadTranslator mOtherSideClosed;
    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;
    private ReadableCanMailbox networkDoorReversal_left;                // reversal
    private ReadableCanMailbox networkDoorReversal_right;
    private DoorReversalCanPayloadTranslator mDoorReversal_left;        // reversal
    private DoorReversalCanPayloadTranslator mDoorReversal_right;
    private int mDesiredDwell = 400;
    private int MAX_Weight = 15000;
    private CallArray callArray;
    // For Car Call
    private CarCallCanPayloadTranslator mCarCall_1_F;
    private CarCallCanPayloadTranslator mCarCall_1_B;
    private CarCallCanPayloadTranslator mCarCall_7_F;
    private CarCallCanPayloadTranslator mCarCall_7_B;
    private ReadableCanMailbox networkCarCall_1_F;
    private ReadableCanMailbox networkCarCall_1_B;
    private ReadableCanMailbox networkCarCall_7_F;
    private ReadableCanMailbox networkCarCall_7_B;
    
    // For Floor 2
    private CarCallCanPayloadTranslator mCarCall_2_B;
    private ReadableCanMailbox networkCarCall_2_B;
    private HallCallCanPayloadTranslator mHallCall_2BU;
    private HallCallCanPayloadTranslator mHallCall_2BD;
    private ReadableCanMailbox networkHallCall_2BU;
    private ReadableCanMailbox networkHallCall_2BD;
    
    // output network
    private WriteableCanMailbox networkDoorMotor;
    private DoorMotorCanPayloadTranslator mDoorMotor;
    // State variables
    private State currentState;
    private int countdown = 0;
    private int waitDispatcher = 0;
    private int Dwell = mDesiredDwell;
    private boolean otherSideOpened = false;
    private boolean otherSideClosed = false;
    
    
    public DoorControl(Hallway hallway, Side side, SimTime period, boolean verbose) {
        super ("DoorControl" + ReplicationComputer.makeReplicationString(hallway, side), verbose);
        
        // set local variables to match constructor parameters
        this.period = period;
        this.side = side;
        this.hallway = hallway;
        this.currentState = State.CLOSED;
        log("Create Door Control with time period = ", period);
        // instantiate physical interface objects
        door_motor = DoorMotorPayload.getWriteablePayload(hallway, side);
        // register the door_motor signal to be sent periodically
        physicalInterface.sendTimeTriggered(door_motor, period);
        
        // instantiate message objects
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        networkDoorOpened_left = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID +
                                                                  ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorOpened_right = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID +
                                                                   ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        networkDoorClosed_left = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                                                                  ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorClosed_right = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                                                                   ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        networkDoorReversal_left = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID +
                                                                    ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        networkDoorReversal_right = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID +
                                                                     ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
        mDoorOpened_left = new DoorOpenedCanPayloadTranslator(networkDoorOpened_left, hallway, Side.LEFT);
        mDoorOpened_right = new DoorOpenedCanPayloadTranslator(networkDoorOpened_right, hallway, Side.RIGHT);
        mDoorClosed_left = new DoorClosedCanPayloadTranslator(networkDoorClosed_left, hallway, Side.LEFT);
        mDoorClosed_right = new DoorClosedCanPayloadTranslator(networkDoorClosed_right, hallway, Side.RIGHT);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        
        // Give DoorOpened and DoorClosed
        if (side.ordinal() == 0) {
        	mDoorOpened = mDoorOpened_left;
        	mDoorClosed = mDoorClosed_left;
        } else {
        	mDoorOpened = mDoorOpened_right;
        	mDoorClosed = mDoorClosed_right;
        }
        //mDoorReversal = new DoorReversalCanPayloadTranslator(networkDoorReversal, hallway, side);
        mDoorReversal_left = new DoorReversalCanPayloadTranslator(networkDoorReversal_left, hallway, Side.LEFT);
        mDoorReversal_right = new DoorReversalCanPayloadTranslator(networkDoorReversal_right, hallway, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDoorOpened_left);
        canInterface.registerTimeTriggered(networkDoorOpened_right);
        canInterface.registerTimeTriggered(networkDoorClosed_left);
        canInterface.registerTimeTriggered(networkDoorClosed_right);
        canInterface.registerTimeTriggered(networkCarWeight);
        //canInterface.registerTimeTriggered(networkDoorReversal);
        canInterface.registerTimeTriggered(networkDoorReversal_left);
        canInterface.registerTimeTriggered(networkDoorReversal_right);
        // output
        networkDoorMotor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID +
                                                             ReplicationComputer.computeReplicationId(hallway, side));
        mDoorMotor = new DoorMotorCanPayloadTranslator(networkDoorMotor, hallway, side);
        canInterface.sendTimeTriggered(networkDoorMotor, period);
        // instantiate an array of AtFloor class
        mAtFloor_array = new AtFloorArray(canInterface);
        callArray = new CallArray(canInterface);
        // set the other side
        if (side.equals(Side.LEFT)) {
        	mOtherSideOpened = mDoorOpened_right;
        	mOtherSideClosed = mDoorClosed_right;
        } else {
        	mOtherSideOpened = mDoorOpened_left;
        	mOtherSideClosed = mDoorClosed_left;
        }
        // register Car Call for floor 1 and 7
        networkCarCall_1_F = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(1, Hallway.FRONT));
        networkCarCall_1_B = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(1, Hallway.BACK));
        networkCarCall_7_F = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(7, Hallway.FRONT));
        networkCarCall_7_B = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(7, Hallway.BACK));
        mCarCall_1_F = new CarCallCanPayloadTranslator(networkCarCall_1_F, 1, Hallway.FRONT);
        mCarCall_1_B = new CarCallCanPayloadTranslator(networkCarCall_1_B, 1, Hallway.BACK);
        mCarCall_7_F = new CarCallCanPayloadTranslator(networkCarCall_7_F, 7, Hallway.FRONT);
        mCarCall_7_B = new CarCallCanPayloadTranslator(networkCarCall_7_B, 7, Hallway.BACK);
        canInterface.registerTimeTriggered(networkCarCall_1_F);
        canInterface.registerTimeTriggered(networkCarCall_1_B);
        canInterface.registerTimeTriggered(networkCarCall_7_F);
        canInterface.registerTimeTriggered(networkCarCall_7_B);
        
        // register Calls for floor 2
        networkHallCall_2BU = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(2, Hallway.BACK, Direction.UP));
        networkHallCall_2BD = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(2, Hallway.BACK, Direction.DOWN));
        networkCarCall_2_B = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(2, Hallway.BACK));
        mCarCall_2_B = new CarCallCanPayloadTranslator(networkCarCall_2_B, 2, Hallway.BACK);
        mHallCall_2BU = new HallCallCanPayloadTranslator(networkHallCall_2BU, 2, Hallway.BACK, Direction.UP);
        mHallCall_2BD = new HallCallCanPayloadTranslator(networkHallCall_2BD, 2, Hallway.BACK, Direction.DOWN);
        canInterface.registerTimeTriggered(networkCarCall_2_B);
        canInterface.registerTimeTriggered(networkHallCall_2BU);
        canInterface.registerTimeTriggered(networkHallCall_2BD);
        
        
        // ready now
        timer.start(period);
    }
    private boolean haveCall(int cur) {
    	// Hall Call
    	if (cur != 2) {
    		if (mDesiredFloor.getDirection().equals(Direction.UP)) {
    			if (callArray.isCalled(cur, Direction.UP, hallway)) {
					return true;
				}
			}
			if (mDesiredFloor.getDirection().equals(Direction.DOWN)) {
				if (callArray.isCalled(cur, Direction.DOWN, hallway)) {
					return true;
				}
			}
    	}
    	if (cur == 2) {
    		if (mDesiredFloor.getDirection().equals(Direction.UP)) {
    			if (mHallCall_2BU.getValue()) {
    				return true;
    			}
    		}
    		if (mDesiredFloor.getDirection().equals(Direction.DOWN)) {
    			if (mHallCall_2BD.getValue()) {
    				return true;
    			}
    		}
    	}
		// Car Call
    	if (cur != 1 && cur != 7 && cur != 2) {
    		if (callArray.isCalled(cur, Direction.STOP, hallway)) {
    			return true;
    		}
    	} else {
    		if (cur == 1 && hallway.equals(Hallway.FRONT)) {
    			return mCarCall_1_F.getValue();
    		} else if (cur == 1 && hallway.equals(Hallway.BACK)) {
    			return mCarCall_1_B.getValue();
    		} else if (cur == 7 && hallway.equals(Hallway.FRONT)) {
    			return mCarCall_7_F.getValue();
    		} else if (cur == 7 && hallway.equals(Hallway.BACK)) {
    			return mCarCall_7_B.getValue();
    		} else if (cur == 2 && hallway.equals(Hallway.BACK)) {
    			return mCarCall_2_B.getValue();
    		}
    	}
    	//System.out.println("Never come here is haveCall function");
    	return false;
    }
    private boolean eitherDoorClosed() {
    	return (mDoorClosed_left.getValue() || mDoorClosed_right.getValue());
    }
    private boolean eitherDoorOpened() {
    	return (mDoorOpened_left.getValue() || mDoorOpened_right.getValue());
    }
	@Override
	public void timerExpired(Object callbackData) {
		State nextState = currentState;
		switch (nextState) {
			case CLOSED:
				Dwell = mDesiredDwell;
				door_motor.set(DoorCommand.STOP);
				mDoorMotor.setDoorCommand(DoorCommand.STOP);
				int desFloor = mDesiredFloor.getFloor();
				int curFloor = mAtFloor_array.getCurrentFloor();
				/**
				 * if other side is closed (to be changed to state)
				 */
				if (mOtherSideClosed.getValue() == true) {
					otherSideClosed = true;
				}
				
//#transition 'T 5.1'
				if (curFloor == desFloor && mAtFloor_array.isAtFloor(curFloor, hallway) && haveCall(curFloor)) {
					if (mDriveSpeed.getSpeed() == 0 && mDriveSpeed.getDirection() == Direction.STOP) {
						//System.out.println("To BEFORE_OPEN From CLOSED");
						nextState = State.BEFORE_OPEN;
					}
				} else if (otherSideClosed == true && mOtherSideClosed.getValue() == false) {
					nextState = State.BEFORE_OPEN;
				}
//#transition 'T 5.6'
				else if ((mDoorReversal_left.getValue() || mDoorReversal_right.getValue()) && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.REVERSE_OPEN;
				}
                else {
					nextState = currentState;
				}
				break;
			case BEFORE_OPEN:
				// set other side
				otherSideClosed = false;
				door_motor.set(DoorCommand.OPEN);
				mDoorMotor.setDoorCommand(DoorCommand.OPEN);
				countdown = Dwell;
//#transition 'T 5.2'
				if (mDoorOpened.getValue()) {
					nextState = State.OPENED;
				} else {
					nextState = currentState;
				}
				break;
			case OPENED:
				door_motor.set(DoorCommand.STOP);
				mDoorMotor.setDoorCommand(DoorCommand.STOP);
				/**
				 * if other side is opened (to be changed to state)
				 */
				if (mOtherSideOpened.getValue() == true) {
					otherSideOpened = true;
				}
				if (otherSideOpened == true && mOtherSideOpened.getValue() == false) {
					nextState = State.CLOSING;
				} else if (countdown > 0) {
					nextState = currentState;
					countdown -= 1;
				}
//#transition 'T 5.3'
				else if (mCarWeight.getWeight() < MAX_Weight) {
					nextState = State.CLOSING;
				} else {
					nextState = currentState;
				}
				break;
			case CLOSING:
				// reset otherSideOpened
				otherSideOpened = false;
				desFloor = mDesiredFloor.getFloor();
				curFloor = mAtFloor_array.getCurrentFloor();
				door_motor.set(DoorCommand.CLOSE);
				mDoorMotor.setDoorCommand(DoorCommand.CLOSE);
				countdown = 0;
				waitDispatcher = 100;
//#transition 'T 5.4'
				if (mDoorClosed.getValue()) {
					nextState = State.WAIT_DISPATCHER;
					//nextState = State.CLOSED;
				}
//#transition 'T 5.5'
				else if (mCarWeight.getWeight() >= MAX_Weight && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.BEFORE_OPEN;
				}
//#transition 'T 5.7'
				else if ((mDoorReversal_left.getValue() || mDoorReversal_right.getValue()) && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.REVERSE_OPEN;
				}
				else {
					nextState = currentState;
				}
				break;
			case REVERSE_OPEN:
				door_motor.set(DoorCommand.OPEN);
				mDoorMotor.setDoorCommand(DoorCommand.OPEN);
				countdown = Dwell;
//#transition 'T 5.8'
				if (mDoorOpened.getValue()) {
					nextState = State.REVERSE_OPENED;
				} else {
					nextState = currentState;
				}
				break;
			case REVERSE_OPENED:
				door_motor.set(DoorCommand.STOP);
				mDoorMotor.setDoorCommand(DoorCommand.STOP);
				/**
				 * check the other side (should be another state)
				 */
				if (mOtherSideOpened.getValue() == true) {
					otherSideOpened = true;
				}
				if (otherSideOpened == true && mOtherSideOpened.getValue() == false) {
					nextState = State.REVERSE_CLOSING;
				}
				else if (countdown > 0) {
					nextState = currentState;
					countdown -= 1;
				}
//#transition 'T 5.9'
				else {
					nextState = State.REVERSE_CLOSING;
				}
				break;
			case REVERSE_CLOSING:
				// reset other side opened
				otherSideOpened = false;
				desFloor = mDesiredFloor.getFloor();
				curFloor = mAtFloor_array.getCurrentFloor();
				countdown = 0;
				door_motor.set(DoorCommand.NUDGE);
				mDoorMotor.setDoorCommand(DoorCommand.NUDGE);
//#transition 'T 5.10'
				if (mDoorClosed.getValue()) {
					//nextState = State.CLOSED;
					nextState = State.WAIT_DISPATCHER;
				}
//#transition 'T 5.11'
				else if ((mDoorReversal_left.getValue() || mDoorReversal_right.getValue()) && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.REVERSE_OPEN;
				}
//#transition 'T 5.12'
				else if (mCarWeight.getWeight() >= MAX_Weight && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.BEFORE_OPEN;
				}
//#transition 'T 5.14'
				else if (curFloor == desFloor && mAtFloor_array.isAtFloor(curFloor, hallway) &&
                         (callArray.isCalled(curFloor, Direction.STOP) || callArray.isCalled(curFloor, Direction.UP, hallway) || callArray.isCalled(curFloor, Direction.DOWN, hallway))) {
					if (mDriveSpeed.getSpeed() == 0 && mDriveSpeed.getDirection() == Direction.STOP) {
						nextState = State.BEFORE_OPEN;
					}
				}
				else {
					nextState = currentState;
				}
				break;
				// add new state
			case WAIT_DISPATCHER:
				door_motor.set(DoorCommand.STOP);
				mDoorMotor.setDoorCommand(DoorCommand.STOP);
				if (mOtherSideClosed.getValue() == true) {
					otherSideClosed = true;
				}
//#transition 'T 5.15'
				if (otherSideClosed == true && mOtherSideClosed.getValue() == false) {
					nextState = State.BEFORE_OPEN;
				}
				else if (waitDispatcher > 0) {
					nextState = currentState;
					waitDispatcher -= 1;
				}
//#transition 'T 5.13'
                else {
					nextState = State.CLOSED;
				}
				break;
			default:
				throw new RuntimeException("State " + currentState + " wasn't recognized");
		}
		if (currentState == nextState) {
			log("remains in state: ", currentState);
		} else {
			log("Transition: ", currentState, " -> ", nextState);
		}
		// update the state variable
		currentState = nextState;
		// report the current state
		setState(STATE_KEY, nextState.toString());
		
		timer.start(period);
	}
    
}

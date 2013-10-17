/*
18649 Fall 2013
Group 17
Qiang Zhang(qiangz)
(other names would go here)
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
import simulator.elevatorcontrol.DriveCommandCanPayloadTranslator;
import simulator.elevatorcontrol.Utility.AtFloorArray;
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
    private DriveCommandCanPayloadTranslator mDriveSpeed;
    private ReadableCanMailbox networkDoorOpened;
    private DoorOpenedCanPayloadTranslator mDoorOpened;
    private ReadableCanMailbox networkDoorClosed;
    private DoorClosedCanPayloadTranslator mDoorClosed;
    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;
    private ReadableCanMailbox networkDoorReversal;                // reversal
    private DoorReversalCanPayloadTranslator mDoorReversal;        // reversal
    private int mDesiredDwell = 100;
    private int MAX_Weight = 14000;
    // output network
    private WriteableCanMailbox networkDoorMotor;
    private DoorMotorCanPayloadTranslator mDoorMotor;
    // State variables
    private State currentState;
    private int countdown = 0;
    private int Dwell = mDesiredDwell;
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
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
        networkDoorOpened = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(hallway, side));
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(hallway, side));
        networkDoorReversal = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(hallway, side));
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        mDriveSpeed = new DriveCommandCanPayloadTranslator(networkDriveSpeed);
        mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        mDoorReversal = new DoorReversalCanPayloadTranslator(networkDoorReversal, hallway, side);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDoorOpened);
        canInterface.registerTimeTriggered(networkDoorClosed);
        canInterface.registerTimeTriggered(networkCarWeight);
        canInterface.registerTimeTriggered(networkDoorReversal);
        // output
        networkDoorMotor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(hallway, side));
        mDoorMotor = new DoorMotorCanPayloadTranslator(networkDoorMotor, hallway, side);
        canInterface.sendTimeTriggered(networkDoorMotor, period);
        // instantiate an array of AtFloor class 
        mAtFloor_array = new AtFloorArray(canInterface);
        // ready now
        timer.start(period);
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
//#transition 'T 5.1'
				if (curFloor == desFloor && mAtFloor_array.isAtFloor(curFloor, hallway)) {
					if (mDriveSpeed.getSpeed() == Speed.STOP ||
							mDriveSpeed.getDirection() == Direction.STOP) {
						nextState = State.BEFORE_OPEN;
					}
				}
//#transition 'T 5.6'
				else if (mCarWeight.getWeight() >= MAX_Weight && mAtFloor_array.isAtFloor(curFloor, hallway)) {
					nextState = State.BEFORE_OPEN;
				} else {
					nextState = currentState;
				}
				break;
			case BEFORE_OPEN:
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
				if (countdown > 0) {
					nextState = currentState;
					countdown -= 1;
				}
//#transition 'T 5.3'
				else {
					nextState = State.CLOSING;
				}
				break;
			case CLOSING:
				door_motor.set(DoorCommand.NUDGE);
				mDoorMotor.setDoorCommand(DoorCommand.NUDGE);
				countdown = 0;
//#transition 'T 5.4'
				if (mDoorClosed.getValue()) {
					nextState = State.CLOSED;
				}
//#transition 'T 5.5'
				else if (mCarWeight.getWeight() >= MAX_Weight && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.BEFORE_OPEN;
				}
//#transition 'T 5.7'
				else if (mDoorReversal.getValue() && mAtFloor_array.isAtFloor(mAtFloor_array.getCurrentFloor(), hallway)) {
					nextState = State.BEFORE_OPEN;
				}
				else {
					nextState = currentState;
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

/*
18649 Fall 2013
Group 17
Qiang Zhang(qiangz)
(other names would go here)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;

public class LanternControl extends Controller{
	// define state variables
	public static enum State {
		Lantern_Off,
		No_On,
		Lantern_On,
	}
	// (in ms) how often this module should send out message
	private final SimTime period;
	private final Direction direction;
	// physical interface
	private WriteableCarLanternPayload car_lantern;
	// input network
	private AtFloorArray mAtFloor_array;
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	private DoorClosedArray mDoorClosed_front_array;
	private DoorClosedArray mDoorClosed_back_array;
	// state variable
	private State currentState;
	Direction DesiredDirection;
	public LanternControl(Direction direction, SimTime period, boolean verbose) {
		super("LanternControl" + ReplicationComputer.makeReplicationString(direction), verbose);
		this.period = period;
		this.direction = direction;
		this.currentState = State.Lantern_Off;
		log("Create CarLantern Control with tim period = ", period);
		// instantiate the physical interface objects
		car_lantern = CarLanternPayload.getWriteablePayload(direction);
		// register the car lantern signal to be sent periodically
		physicalInterface.sendTimeTriggered(car_lantern, period);
		
		// instantiate message objects
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		// instantiate the door closed array
		mDoorClosed_front_array = new DoorClosedArray(Hallway.FRONT, canInterface);
		mDoorClosed_back_array = new DoorClosedArray(Hallway.BACK, canInterface);
		mAtFloor_array = new AtFloorArray(canInterface);
		// ready now
		timer.start(period);
	}
	
	@Override
	public void timerExpired(Object callbackData) {
		// cases Lantern_Off, No_On, Turn_Up_On, Turn_Down_On
		State nextState = currentState;
		boolean allClose;
		switch (nextState) {
		case Lantern_Off:
			car_lantern.set(false);
			allClose = mDoorClosed_front_array.getBothClosed() && mDoorClosed_back_array.getBothClosed();
			// code for desired direction for this simple design
                	//wxl
                	if(mDesiredFloor.getFloor()==mAtFloor_array.getCurrentFloor()){
                    	    if (allClose) {
                                DesiredDirection = mDesiredFloor.getDirection();
                            }
                	}else if(mDesiredFloor.getFloor()>mAtFloor_array.getCurrentFloor()){
                    		DesiredDirection=Direction.UP;
                	}else{
                    		DesiredDirection=Direction.DOWN;
                	}
				
				// should be DesiredDirection = mDesiredFloor.getDirection();
//#transition 'T 7.1'
					if (allClose == false && DesiredDirection == Direction.STOP) {
						nextState = State.No_On;
						break;
					}
//#transition 'T 7.2'
					else if (allClose == false && DesiredDirection == direction) {//wxl && mDesiredFloor.getFloor() != mAtFloor_array.getCurrentFloor()) {
						nextState = State.Lantern_On;
						break;
					}
					else {
						nextState = State.Lantern_Off;
					}
				break;
			case No_On:
				car_lantern.set(false);
				allClose = mDoorClosed_front_array.getBothClosed() && mDoorClosed_back_array.getBothClosed();
//#transition 'T 7.3'
				if (allClose) {
					nextState = State.Lantern_Off;
				}
				else {
					nextState = State.No_On;
				}
				break;
			case Lantern_On:
				car_lantern.set(true);//wxl
				allClose = mDoorClosed_front_array.getBothClosed() && mDoorClosed_back_array.getBothClosed();
//#transition 'T 7.4'
				if (allClose) {
					nextState = State.Lantern_Off;
				}
				else {
					nextState = State.Lantern_On;
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

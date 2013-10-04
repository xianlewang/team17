package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
import simulator.framework.Controller;

public class CarButtonControl extends Controller{
	/***************************************************************************
     * Declarations
     **************************************************************************/
    //note that inputs are Readable objects, while outputs are Writeable objects
	//inputs
	//network interface
	//received AtFloor message
    private ReadableCanMailbox networkAtFloor;
    //translator for the AtFloor message
    private AtFloorCanPayloadTranslator mAtFloor;
    //received desired floor message
    private ReadableCanMailbox networkDesiredFloor;
    //translator for the desired floor message
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    //received door closed message
    private ReadableCanMailbox networkDoorClosed;
    //translator for the doorClosed message -- this translator is specific
    //to this messages, and is provided the elevatormodules package
    private DoorClosedCanPayloadTranslator mDoorClosed;
    //local physical state
    private ReadableCarCallPayload CarCall;
    // output
    //local physical state
    private WriteableCarLightPayload CarLight;
    //network interface
    //receive car light
    private WriteableCanMailbox networkCarLight;
    //translator for the car light message -- this is a generic translator
    private BooleanCanPayloadTranslator mCarLight;
    // receive car call from the other button
    private WriteableCanMailbox networkCarCall;
    //translator for the car call message -- this is a generic translator
    private BooleanCanPayloadTranslator mCarCall;
    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final int floor;
    //store the period for the controller
    private SimTime period;
    //enumerate states
    private enum State{
    	STATE_LIGHT_OFF,
    	STATE_LIGHT_ON
    }
    //state variable initialized to the initial state STATE_LIGHT_OFF
    private State state = State.STATE_LIGHT_OFF;
    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     */
    public CarButtonControl(SimTime period, int floor, Hallway hallway, boolean verbose){
    	//call to the Controller superclass constructor is required
    	super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway), verbose);
    	//stored the constructor arguments in internal state
    	this.period = period;
    	this.floor = floor;
    	this.hallway = hallway;
    	
    	log("Created CarButtonControl with period = ", period);
    	// initialize input
    	//create a payload object for this floor,hallway using the
        //static factory method in CarCallPayload.
    	CarCall = CarCallPayload.getReadablePayload(floor, hallway);
    	//register the payload with the physical interface (as in input) -- it will be updated
        //periodically when the car call button state is modified.
    	physicalInterface.registerTimeTriggered(CarCall);
    	//create a can mailbox - this object has the binary representation of the message data
        //the CAN message ids are declared in the MessageDictionary class.  The ReplicationComputer
        //class provides utility methods for computing offsets for replicated controllers
    	networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + 
    			ReplicationComputer.computeReplicationId(floor, hallway));
    	mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
    	//register the mailbox to have its value broadcast on the network periodically
        //with a period specified by the period parameter.
    	canInterface.registerTimeTriggered(networkAtFloor);
    	
    	networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
    	mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
    	canInterface.registerTimeTriggered(networkDesiredFloor);
    	
    	networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
    			ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
    	mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, Side.LEFT);
    	canInterface.registerTimeTriggered(networkDoorClosed);
    	
    	// initialize output
    	CarLight = CarLightPayload.getWriteablePayload(floor, hallway);
    	physicalInterface.sendTimeTriggered(CarLight, period);
    	
    	networkCarLight = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_LIGHT_BASE_CAN_ID + 
    			ReplicationComputer.computeReplicationId(floor, hallway));
    	mCarLight = new BooleanCanPayloadTranslator(networkCarLight);
    	canInterface.sendTimeTriggered(networkCarLight, period);
    	
    	networkCarCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + 
    			ReplicationComputer.computeReplicationId(floor, hallway));
    	mCarCall = new BooleanCanPayloadTranslator(networkCarCall);
    	canInterface.sendTimeTriggered(networkCarCall, period);
    	
    	timer.start(period);
    }
    
    public void timerExpired(Object callbackData) {
    	State nextstate = state;
    	switch (state) {
    	case STATE_LIGHT_OFF:
            //state actions for 'STATE_LIGHT_OFF'
    		CarLight.set(false);
    		mCarLight.set(false);
    		mCarCall.set(false);
//#transition 'T9.1'
    		if (CarCall.pressed() == true) {
    			nextstate = State.STATE_LIGHT_ON;
    		} else {
    			nextstate = state;
    		}
    		break;
    	case STATE_LIGHT_ON:
            //state actions for 'STATE_LIGHT_ON'
    		CarLight.set(true);
    		mCarLight.set(true);
    		mCarCall.set(true);
//#transition 'T9.2'
    		if (mAtFloor.getValue() == true && mDesiredFloor.getFloor() == floor) {
    			nextstate = State.STATE_LIGHT_OFF;
    		} else {
    			nextstate = state;
    		}
    		break;
    	default:
            throw new RuntimeException("State " + state + " was not recognized.");
    	}
    	//log the results of this iteration
        if (state == nextstate) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",nextstate);
        }

        //update the state variable
        state = nextstate;

        //report the current state
        setState(STATE_KEY,nextstate.toString());

        //schedule the next iteration of the controller
        //you must do this at the end of the timer callback in order to restart
        //the timer
        timer.start(period);
    }
}

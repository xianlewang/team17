package simulator.elevatorcontrol;

import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class CarLightCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public CarLightCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
        super(payload, MessageDictionary.CAR_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway),
        	  "CarLight" + ReplicationComputer.makeReplicationString(floor, hallway));
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public CarLightCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
    	super(payload, MessageDictionary.CAR_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), 
    		  "CarLight" + ReplicationComputer.makeReplicationString(floor, hallway));
    }
	
}

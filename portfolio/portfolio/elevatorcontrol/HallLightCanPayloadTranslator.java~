package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class HallLightCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public HallLightCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, MessageDictionary.HALL_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), 
        	  "HallLight" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public HallLightCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
    	super(payload, MessageDictionary.HALL_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), 
    		  "HallLight" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }
	
}

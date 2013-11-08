package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class CarLanternCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public CarLanternCanPayloadTranslator(WriteableCanMailbox payload, Direction direction) {
        super(payload, MessageDictionary.CAR_LANTERN_BASE_CAN_ID + ReplicationComputer.computeReplicationId(direction), 
        	  "CarLantern" + ReplicationComputer.makeReplicationString(direction));
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public CarLanternCanPayloadTranslator(ReadableCanMailbox payload, Direction direction) {
    	super(payload, MessageDictionary.CAR_LANTERN_BASE_CAN_ID + ReplicationComputer.computeReplicationId(direction), 
          	  "CarLantern" + ReplicationComputer.makeReplicationString(direction));
    }
	
}

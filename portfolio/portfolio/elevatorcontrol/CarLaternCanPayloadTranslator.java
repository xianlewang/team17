/**
* 18649 Fall 2013
* Group 17
* Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
**/

package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class CarLaternCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public CarLaternCanPayloadTranslator(WriteableCanMailbox payload, Direction direction) {
        super(payload, MessageDictionary.CAR_LANTERN_BASE_CAN_ID + ReplicationComputer.computeReplicationId(direction), 
        	  "CarLatern" + ReplicationComputer.makeReplicationString(direction));
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public CarLaternCanPayloadTranslator(ReadableCanMailbox payload, Direction direction) {
    	super(payload, MessageDictionary.CAR_LANTERN_BASE_CAN_ID + ReplicationComputer.computeReplicationId(direction), 
          	  "CarLatern" + ReplicationComputer.makeReplicationString(direction));
    }
	
}
/*
 * 18649 Fall 2013
 * Group 17
 * Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
 */ 
package simulator.elevatorcontrol;
import java.util.BitSet;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Utility superclass for CAN payload translators with a single boolean value.
 * This class consolidates most of the functionality required for a boolean
 * translator so that most of the actual translators used by system objects
 * can be specified by a set of constructor parameters in the BooleanCanTranslator constructor.
 *
 * This class is intentionally not available to classes in simulator.elevatorcontrol.
 * You are welcome to implement a similar generic class if you want to.
 *
 * Note that this translator and the ones descended from it are not compatible
 * with simulator.payloads.BooleanCanPayloadTranslator.  The same translator used
 * to encode a value should also be used to read it.
 *
 * @author justinr2
 */
public class GenericBooleanTranslator extends CanPayloadTranslator {

    String name;

    GenericBooleanTranslator(WriteableCanMailbox payload, int expectedMask, String name) {
        super(payload, 1, expectedMask);
        this.name = name;
    }
    
    GenericBooleanTranslator(ReadableCanMailbox payload, int expectedMask, String name) {
        super(payload, 1, expectedMask);
        this.name = name;
    }

    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }

    public void setValue(boolean value) {
        BitSet b = getMessagePayload();
        b.set(0, value);
        setMessagePayload(b, getByteSize());
    }

    public boolean getValue() {
        return getMessagePayload().get(0);
    }

    @Override
    public String payloadToString() {
        return name + " = " + getValue();
    }
    }
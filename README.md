# SpurButton

## Messages

All messages between nodes and bridges follow the same format consisting 10 bytes of headers and a variable length payload (some messages do not have a payload):

![Message Format](https://github.com/ContinuumBridge/SpurButton/blob/master/Pictures/Message_Format.jpg)

All messages are acknowledged in both directions. 

No new messages between a communicating bridge and node are sent until the previous message has been acknowledged.

All words are sent in big endian order (MS byte first).

## Field Descriptions

**Message Source and Destination Addresses**

A two-byte address indicating the source and destination of a message. 

Nodes get their address by being added to a network (included).

Once it has an address, a node will discard any message that does not have its address (or the beacon address, see below) in the destination field. 

Note that the destination field is never encrypted. The rest of the message, including the source address, will be encrypted. 

**Message Function Code**

All function codes defined to-date are shown in Table 1: Message Function Code Definitions at the end of this section.

**Total Message Length**

This unsigned integer is the total message length in bytes including all headers. The total length of any message therefore cannot exceed 256 bytes.

**Message Payload**

The variable length payload takes various forms depending on the source and the message function code.
Note that all messages from a bridge to a node include a “time to next wake-up” field which is sent as the first two bytes of the payload:

**Message Function Code Definitions**

|Message Function Code | Source | Name |Description |Used during |
|----------------------|--------|------|------------|------------|
|0x00 | Node | Button Include Req |Request from a button node to be added to a network identified in a beacon message from a bridge | Network inclusion |
|0x01 | Node | Sensor Include Req | Request from a sensor node to be added to a network identified in a beacon message from a bridge | Network inclusion |
|0x02 | Bridge | Include Grant | Response from the bridge to an include request from a node. This assigns a new address to the node | Network inclusion|
|0x05 | Bridge | Configuration | Node configuration command | Normal operation |
|0x06 | Bridge | Send Battery Status | Requests node to send battery status | Noraml operation |
|0x07 | Node | Woken Up | Indication from a node that it has woken up and the bridge may communicate with it| Normal operation |
|0x08 | Either | Ack | All messages are acknowledged in both directions | Always |
|0x09 | Node | Alert | A node sends an alert to its associated bridge | Normal operatiion |
|0x0A | Bridge | Beacon | Beacon messages are broadcast periodically by a bridge. Nodes use these messages to identify local networks. A beacon message is never encrypted | Always |
|0x0B | Bridge | Start | Sent after configuration download is complete | Always |

All function codes are fully described in the relevant sections below. These are split into Generic, Network Inclusion (adding a node to a network) and Normal Operation.

## Button Configuration Meesages

This section defines the payload of configuration messages.

**Display Screen Definitions**

These are sent in configuration messages. Up to 32 screens may be defined. Each screen can have up to two regions and the definition of each region is limited to 128 bytes. Each region is downloaded to the button in a separate configuration message. 

Screen definitions consist of a series of key characters, each followed by one or more bytes. For example, Sx is the start of the definition for screen x, where x is a 1-byte unsigned integer. All the possible operations are dfined below, where each character and parameter is a single byte:

    Sx Screen x
    Rx Region x
    Fx Font x
    Xx Set X position to x
    Yy Set Y position to y
    Lpq Draw a line from (x, y) to (p, q)
    Bwh Draw a box starting at (x, y), width w, height h
    Tx String of length x follows
    Cx Centred string of length x follows
    ER End of region
    ES End of screen. There is no ER before this

Here is an example in C format:

    "F\x03" "Y\x10" "C\x0C" "Push here to\x00" "Y\x32" "C\x0E" "report a fault\x00" "ES"
    
and a more complex example, which includes drawing two boxes:
    
    "F\x02" "X\x18" "Y\x0E" "T\x04" "Push\xFF" "X\x08" "Y\x24" "T\x08" "here for\xFF"
    "X\x20" "Y\x3C" "T\x04" "bill\xFF" "F\x02" "X\x7D" "Y\x0E" "T\x04" "Push\xFF" "X\x6D" "Y\x24"
    "T\x08" "here for\xFF" "X\x70" "Y\x3C" "T\x07" "service\xFF"
    "X\x02" "Y\x02" "B\x5A\x5C" "X\x03" "Y\x03" "B\x58\x5A"
    "X\x68" "Y\x02" "B\x5A\x5C" "X\x69" "Y\x03" "B\x58\x5A" "ES"
    
Note that although the length of each string is given, it is still terminated by a null character, 0x00.

**General Purpose State Machine**

As the character S is already used for a screen definition, M, as in machine, is used for state machine definitions. A 16-byte state machine definition is a aseries of fields, defined below, preceeded by the character M.

The button function is determined by a general-purpose state machine. There is a maximum of 32 states, the last four of which are
reserved for system use (28, 29, 30, 31), leaving 28 for a user program. The following values are programmed for each state (each is a single byte).

| Byte  |  Name  |   Use    |
|-------|--------|----------|
|  0     | S     | State number |
|  1     | D     | Display screen number to display on entry. 0xFF = don't change |
|  2     | A     | Alert to send on entry. 0xFF = don't send alert |
|  3     | LD    | State to go to on left double-push. 0xFF = ignore |
|  4     | LS    | State to go to on left single-push. 0xFF = ignore |
|  5     | MS    | State to go to on middle single-push (left & right together). 0xFF = ignore |
|  6     | MD    | State to go to on middle double-push (left & right together). 0xFF = ignore |
|  7     | RS    | State to go to on right single-push. 0xFF = ignore |
|  8     | RD    | State to go to on right double-push. 0xFF = ignore |
|  9     | XV    | When button receives this value, go to state RS. 0xFF = ignore |
|  A     | XS    | State to go to when value RV is received |
|  B     | W     | Wait in this state for W seconds before going to state DWS. 0xFF = igore |
|  C     | WS    | Go to this state after W seconds |
|  D     | -     | Reserved for future use |
|  E     | -     | Reserved for future use |
|  F     | -     | Reserved for future use |

**Simple example**

Here is a simple example. This is the "standard" application:

- In state 1, a single push takes us to state 2 and message 2 is displayed.
- In state 2, a double push takes us back to state 1 and message 1 is displayed.
- A config message of 01 takes us to state 3, regardless of starting state, and message 3 is displayed.
- In state 3, a double push takes us back to state 1.
- State 0 is the starting state. It's the same as state 1, except that no message is sent on entering it.

The state table is as follows:

| S | D | A | LD | LS | MS | MD | RS | RD | XV | XS | W | WS |
|---|---|---|----|----|----|----|----|----|----|----|---|----|
| 0 | 1 | FF| FF | 02 | 02 | FF | 02 | FF | 01 | 3  | FF| FF |
| 1 | 1 | 01| FF | 02 | 02 | FF | 02 | FF | 01 | 3  | FF| FF |
| 2 | 2 | 02| 01 | FF | FF | 01 | FF | 01 | 01 | 3  | FF| FF |
| 3 | 3 | FF| 01 | FF | FF | 01 | FF | 01 | 00 | 1  | FF| FF |

An example use of this is with the following screens:

* Screen 1: "Push here if this machine requires more coffee"
* Screen 2: "More coffee for this machine has been requested"
* Screen 3: "Coffee will be replenished by 14:30" (as updated from the server)


**Example with left and right options and delays**

In this example, a message is briefly displayed after a button push. This could, for example, say: "Your request is being sent".

- In state 1, a left-push takes us to state 2 and a right-push to state 3.
- In state 2, screen 2 is displayed for 3 seconds and alert 2 is sent, then we move to state 4.
- In state 3, screen 2 is displayed for 3 seconds and alert 3 is sent, then we move to state 5.
- In state 4, screen 3 is displayed until there is a double-push, then we revert to state 1.
- In state 5, screen 4 is displayed until there is a double-push, then we revert to state 1.

The state table is as follows. Again, the starting state, 0 is the same as 1, but no message is sent when it is entered.

| S | D | A | LD | LS | MS | MD | RS | RD | XV | XS | W | WS |
|---|---|---|----|----|----|----|----|----|----|----|---|----|
| 0 | 1 | FF| FF | 02 | FF | FF | 03 | FF | FF | FF | FF| FF |
| 1 | 1 | 01| FF | 02 | FF | FF | 03 | FF | FF | FF | FF| FF |
| 2 | 2 | 02| FF | FF | FF | FF | FF | FF | FF | FF | 03| 04 |
| 3 | 2 | 03| FF | FF | FF | FF | FF | FF | FF | FF | 03| 05 |
| 4 | 3 | FF| 01 | FF | FF | 01 | FF | 01 | FF | FF | FF| FF |
| 5 | 4 | FF| 01 | FF | FF | 01 | FF | 01 | FF | FF | FF| FF |

An example use of this is with the following screens:

* Screen 1: "Push here if supplies low | Push here if not working"
* Screen 2: "The problem is being reported"
* Screen 3: "Low printer supplies reported"
* Screen 4: "Fault with this printer has been reported"

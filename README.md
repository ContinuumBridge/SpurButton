# SpurButton

## General Purpose State Machine

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

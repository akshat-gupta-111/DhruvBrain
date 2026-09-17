# DhruvBrain Command Protocol (BLE)

This document outlines the BLE command strings that the Jetson Nano (Central) sends to the Arduino Uno R4 (Peripheral) via the string characteristic `19b10001-e8f2-537e-4f6c-d104768a1214`.

## LED Control Commands (Moods)
These commands instantly change the behavior of the 12V Blue LED connected to the Arduino.

| Command String | Behavior | Description |
| :--- | :--- | :--- |
| `LED:IDLE_WHITE` | Solid ON | Dhruv is waiting and observing passively. |
| `LED:CURIOSITY_GREEN` | Slow pulsing fade | Dhruv has noticed something interesting and is analyzing it. |
| `LED:FLIRT_PINK` | Double blink pattern | Dhruv sees a human and is delivering a witty/flirty remark. |
| `LED:ALERT_RED` | Fast aggressive strobe | Dhruv detects an obstacle or error state. |
| `LED:THINKING_BLUE` | Rapid random flickering | Dhruv is actively processing audio/LLM inference. |

## Motor Control Commands (Mecanum 8-Way Drive)
Motor commands are formatted as `MOTOR:<DIR>:<SPEED>:<TIME_MS>`.
Example: `MOTOR:STRAFE_L:200:1500` (Strafe left at speed 200 for 1.5 seconds).

*Note: For true mecanum movement, all 4 wheels must be independently driven using 8 pins (or a motor shield).*

| Command String `<DIR>` | AI Action Mapping | Description (Wheel Directions) |
| :--- | :--- | :--- |
| `FWD` | `APPROACH_0.5M` | All wheels spin Forward. |
| `REV` | `BACKUP_0.5M` | All wheels spin Backward. |
| `STRAFE_L` | `STRAFE_LEFT` | FL Backward, BL Forward, FR Forward, BR Backward. |
| `STRAFE_R` | `STRAFE_RIGHT` | FL Forward, BL Backward, FR Backward, BR Forward. |
| `DIAG_FL` | `DIAGONAL_FL` | FL Stopped, BL Forward, FR Forward, BR Stopped. |
| `DIAG_FR` | `DIAGONAL_FR` | FL Forward, BL Stopped, FR Stopped, BR Forward. |
| `PIVOT_L` | `PIVOT_LEFT_30` | Left wheels Backward, Right wheels Forward. |
| `PIVOT_R` | `PIVOT_RIGHT_30` | Left wheels Forward, Right wheels Backward. |
| `STOP` | `HALT` | All wheels stop immediately. |

## Complete Flow Example
1. User speaks: "Who are you?"
2. Jetson sends: `LED:THINKING_BLUE`
3. LLM decides to flirt and approach.
4. Jetson sends: `LED:FLIRT_PINK`
5. Jetson sends: `MOTOR:FWD:255:500`
6. Arduino executes motor movement and automatically stops after 500ms.

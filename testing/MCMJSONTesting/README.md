# MCM JSON Testing

This folder contains example JSON files used to test MCM (Manoeuvre Coordination Message)
triggering via the OScar JSON server interface.

Each file represents a valid `MCM_trigger` request targeting a different MCM container
variant, as defined by the ETSI MCM standard. They can be sent to the OScar JSON server
using a socket client (e.g. netcat):

```bash
nc -q 1 <host> <port> < example_vmc.json
```

---

## Common Fields (all examples)

Every request must include the following top-level fields:

| Field             | Description                                              |
|-------------------|----------------------------------------------------------|
| `request_type`    | Must be `"MCM_trigger"`                                  |
| `MCMType`         | MCM message type (integer)                               |
| `MCMManeuverID`   | Unique maneuver identifier                               |
| `MCMITSRole`      | Role of the ITS station                                  |
| `MCMStationType`  | Station type (e.g. `3` = vehicle)                        |
| `MCMStationID`    | Station identifier                                       |
| `MCMGoal`         | Manoeuvre cooperation goal (use instead of `MCMCost`)    |
| `MCMCost`         | Manoeuvre cooperation cost (use instead of `MCMGoal`)    |

> At least one of `MCMGoal` or `MCMCost` must be present.  
> If `MCMType` is `4` or `7`, `MCMExecutionStatus` is also required.

---

## Files

### `example_vmc.json` — Vehicle Manoeuvre Container
The most complete example. Includes:
- One submaneuver with temporal characteristics, strategy, reference trajectory
  (waypoints, speed, heading, absolute positions), and a target road resource (TRR)
  with lane info and geometry.
- An optional manoeuvre advice list embedded in the container, with advised
  trajectory and advised TRR per submanoeuvre.

### `example_mac.json` — Manoeuvre Advice Container
Standalone advice from one ITS station to another (executant). Includes:
- Executant ID and current state advised change.
- One submanoeuvre with advised trajectory and advised TRR (with temporal
  characteristics).

### `example_rc.json` — Response Container
A response (accept/decline) to a previously received manoeuvre proposal. Includes:
- `MCResponse`: `0` = accept, `1` = decline.
- `MCDeclineReason`: required when declining.
- Optional submaneuvers with trajectory and TRR details.

### `example_ac.json` — Acknowledgment Container
A minimal acknowledgment of a received MCM. Includes:
- `MCAcknowledgmentType`: the type of message being acknowledged.

### `example_tc.json` — Termination Container
The simplest possible MCM. Signals termination of a manoeuvre coordination session.
No container-specific fields are required beyond the common header fields.

---

## Notes on Field Constraints

Some fields have ASN.1-defined value ranges that must be respected:

| Field          | Constraint         |
|----------------|--------------------|
| `LaneCount`    | `0..31`            |
| `TrrWidth`     | `0..15`            |
| `WayPoints`    | min. 2 elements    |

Violating these will cause an ASN.1 UPER encoding failure.
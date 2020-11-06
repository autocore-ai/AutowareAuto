avp_web_interface {#avp_web_interface-package-design}
===========

This is the design document for the avp_web_interface package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
To provide a high-level GUI for the autonomous valet parking demonstration.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

This package is a basic HTML webpage to provide two options to the user:

1. Request the autonomous vehicle to park in a designated area.
2. Request the autonomous vehicle to return to the drop-off area.

For each option, there is a button. Upon clicking, a message is published to set a hard-coded goal
pose. The communication is handled in javascript through `roslibjs`.

## Assumptions / Known limits
<!-- Required -->
The web page does not use any CSS to look pretty.

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
    
To run the interface, two dependencies have to met. Preferanbly both are launched together with the
rest of the stack with the help of `launch.actions.ExecuteProcess`.

### web server

To serve the web interface, a web server has to run on port 8000 so a user can connect, for example
from the localhost at http://127.0.0.1:8000/

As server load is low, a simple 

    python3 -m http.server 8000

is enough when executed in the directory where `index.html` is stored.

### rosbridge

To translate commands from javascript into messages in the ROS2 world, a bridge is needed. An
easy way to achieve that is to use the `rosbridge_server` ROS2 package and to execute
`rosbridge_websocket`.

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

This package is ****for demonstration purposes** only. It has no authentication, so once things are
running, anyone with access to the network could issue a command to move the vehicle.

# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
<!-- Required -->

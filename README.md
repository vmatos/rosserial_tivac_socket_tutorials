# rosserial_tivac_socket_tutorials
Tutorials and examples for rosserial_tivac_socket package.

Take advange of rosserial_tivac_socket for a full catkanized development environment for your rosserial projects,
on the Tiva C Connected Launchpad over TCP/IP sockets.

Note: Include directory inclusion makes include files on the application directory to have precedence.

## Tutorials and examples:

### chatter
Publishes on topic `/chatter` a string every few milliseconds.

### buttons
Publishes on topic `/button_state` the current state of the user switches.

### freertos
Example of an application using rosserial_tivac_socket with freertos.
A new rosserial hardware class is used to deal with the introduction of the RTOS.
A custom startup_gcc.c file is used to include the new RTOS interrupts.
Different lwipopts.h are used.

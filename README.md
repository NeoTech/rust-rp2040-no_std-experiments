# RUST no_std Experiments for RP2040.
Goals is to make a simple no_std program that can be run on the RP2040.
That listens to a USB serial interface and executes commands.
In the long run i want it to execute its commands on the second core.
Leaving the first core to just handle the USB Serial interface and command delegation.

## The files.
This repo containers a boilerplate.rs file that is a simple no_std program that can be run on the RP2040. And just blinks the LED, but its written with some excess code to make it easy to understand and expand on.

The usbled.rs is where i'm experimenting with expanding on the USB serial interface. One caveat with the usb-device is that it needs to poll the host every 10ms, which means you cannot block that call for very long doing other things. That is why im planning to move commands onto the second core.
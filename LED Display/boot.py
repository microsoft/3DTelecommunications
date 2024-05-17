#this code runs at power-on
import usb_cdc
#Here I'm setting up the Trinket to have two UARTs on the USB plug, one for the console, one for data.  If you don't need the console, consider
#setting console=False so there's only one UART when you plug it in, and code that needs to send data to the trinket doesn't get confused
#if you need to do debugging, set console=True and open up the other UART in a terminal to see CircuitPython output and use REPL
usb_cdc.enable(console=True, data=True)
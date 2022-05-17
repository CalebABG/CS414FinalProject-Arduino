# Project Breakdown

Here's the in depth details for the phone app as well as the Arduino driver code. This will likely be a lot, so sit down, grab some snacks, and enjoy the read! 🍿🥤🎉

---

## **Android App**

The Android app makes use of the phones built-in accelerometer for controlling the movement of the car. The app registers and then listens for accelerometer sensor changes, and then transmits packets for the Arduino to drive the motors through Bluetooth.

`SharedPreferences` is used for saving of appsettings / app state. In the app you can set drive parameters which scale the drive speed and turning speed of the bot, as well as a safety button which acts as parental control over the bot. 

As the app uses Bluetooth, Android requires that Bluetooth permissions be granted as runtime permissions along with Location permissions as use of Bluetooth may search for nearby devices. 

The app is composed of multiple activities which make up its functionality: 
1. Start screen which prompts to grant `Bluetooth` and `Location` permissions
2. Control activity which allows for controlling the bot
3. Save replay activity which allows for saving `real-time` Arduino packet recordings to an on-device `NoSQL` database
4. View replays activity which allows the user to send, stop, and delete real-time Arduino `packet recordings`.

### **Control Activity**

In the Control activity there are two control sections, on the left side there are panels which allow one to view the raw or calculated accelerometer values. The next panel comprises of buttons. The first of which is an enable toggle button which enables or disables remote control of the car. The second button acts as an emergency stop button which will immediately stop the motors each time the button is pressed. The final button in this panel is a Bluetooth connect button, which establishes a connection between the phone and the Arduino Bluetooth module. The last panel in the left section has a TextView which displays the Bluetooth connection status, as well as a button on the right of the TextView which allows for terminating the connection.

The second section on the right is wider, comprised of two panels. The first panel is for setting the bots drive parameters. The drive parameters are: drive speed (forwards and backwards), and turning speed (veering left or right). These parameters are used directly on the Arduino, no pre-scaling happens on the phone side, these values are sent as a dedicated packet which updates the same set of parameters on the Arduino. The last panel allows for real-time packet recording and saving. There are buttons for starting recording, stopping, saving, and viewing all saved recordings. At the time of writing this report, the app allows up to 350 packets or around 20-25 seconds of packet recording. Once you've started to record packets, the counter will increment and display the current number of recorded packets. The recording will automatically stop once the maximum number of packets have been recorded, otherwise you can manually stop the recording. Once stopped, you can choose to save or discard the recording. You can choose to give the recording a name or not. If you leave it blank, the app will generate a name for you in the form of: "replay_" plus a unique identifier. If there's a recording with the same name, the app will append the first 6 characters of a unique identifier to the end of the name you've given. These movement recordings come in surprisingly handy in debugging and are just fun!

### **View Replays Activity**

In the View Replays activity you're able to see all of the packet Replays you've saved in a scrollable list on the left side of the screen. On the right, you've got buttons which let you either start the playback / send of the replay you've selected; stop the replay playback in progress, or delete entirely the selected replay. These buttons are state aware, so if you've started the playback of a replay, you will not be able to delete the same replay as it's being sent, you will have to first stop playback and then delete it.

### **Issues / Future Work + Improvements**

Currently in the app there is no mechanism for auto re-connect after a connection issue or failure, as well as an issue with the Bluetooth service not noticing or timing out dead connections. For example, if the phone connects to the bot initially, then the power is disconnected from the Arduino on the bot, the connection is physically cut, but the app will still display a connected status.

Handling connection disconnects and automatic reconnect are definitely improvements which can be made.

---

## **Arduino Driver**

On the Arduino side, one of the first things on the todo list was to design a viable message protocol for the Arduino and Phone to use via Bluetooth.

For the design of the message protocol, I went with a very simple packet format. The packet format consists of:

1. Start of Packet = `UInt8`: 0x1
2. Packet ID = `UInt16`
3. Packet CRC = `UInt32`
4. Packet ACK = `UInt8`
5. Packet Data Length = `UInt16`
6. Packet Data = `UInt8Array`
7. End of Packet = `UInt8`: 0x4

Although fairly simple, I wanted to try to make the format as flexible as possible within the time constraints that I had. The packet ID being a `UInt16` (2 byte) value made sense because this would allow over 65 thousand message types to be defined. While most likely overkill for this project, having the extra room for future uses is always good.

The `CRC` was added for checking packet data integrity. If anything goes wrong or is corrupted for any reason via transmission or in processing on either the phone or the Arduino, calculating the packets CRC and checking if the packets CRC matches the computed CRC acts as a data integrity check, ensuring that only packets whose CRC's match will be processed.

The `ACK` was added as a safety measure while using the phone as the primary controller of the bot. There is a safety function which runs in the Arduino's while loop which checks whether an ACK has been seen/received after an interval of time. If no ACK has been received, then the Arduino sends a command to immediately stop the motors.

The `data` array for the packet is set to 16 bytes in the Arduino file defining the packet, but this can be easily changed; I’ve tested up to 64 bytes and the protocol held up fine. 

With the message protocol and packet format designed and out of the way, now the fun part can begin, sending data from the phone. Or well, in the Arduino's case, receiving and interpreting that data!

To accomplish this, making use of a state-machine was key. Using the state-machine allows for the needed sequence checking of incoming bytes and verification that the bytes received are the correct bytes for the corresponding packet's fields. It processes any data from the Bluetooth module byte by byte, and fills in the data for the packet. As of writing this report, the state-machine's loop takes three arguments: the data byte read from the Bluetooth module, the pointer to the packet to fill with data, and a callback function to execute upon successful packet processing.

With that, we've got the two needed parts in order: a way to send data between the phone and the Arduino, and a way for the Arduino to receive and process incoming data... well, mostly. One of the crucial pieces that's missing in the processing of the incoming data is endianness! Endianness is the byte-order of the data or put another way, is the order in which the bytes of data are read. What's important here is that there are two common forms of byte order: `Little Endian` and `Big Endian`. In `Little Endian` byte-ordered machines, the **least** significant byte of data is stored at the smallest memory address. Whereas in `Big Endian` machines, the **most** significant byte of data is stored at the smallest memory address.

This piece of the puzzle is critical because for any data transmitted that spans more than 1 byte, composing the data back to its correct representation / value depends on the endianness of the Arduino, since it needs to interpret it and potentially make use of the data (driving the motors as an example!). Fortunately, Arduino forum posts gives us the needed architecture information - [Arduino Endianness](https://forum.arduino.cc/t/little-endian-or-big-endian/41382), it's Little Endian!

The second crucial piece of processing is concerned with the Arduino itself. Since at the time of writing this report, the packet format is composed of Integer data type fields, it's important to know as well as specify how many bits or bytes each field will take when processing incoming data. Fortunately, since Arduino is written in `C++`, we have access to numerous integer data types. Arduino makes it easy as there's an integer data type `int` and `unsigned int` (only storing positive values). The tricky part here is that while the data type definitions are convenient to declare, the number of bits / bytes they occupy change depending upon the board you have. As pointed out by the reference - [Int Datatype](https://www.arduino.cc/en/reference/int), `int` occupies 16 bits or `2 bytes` on Arduino UNO and other ATmega based boards, while on the Arduino Due and SAMD based boards, it occupies 32 bits or `4 bytes`. 

Due to this (no pun intended), being more explicit in the data types used is needed because only allocating the needed number of bytes to hold something based on the packet format will not only cut down on the memory required, but also the performance of processing incoming data (albeit most of the time not noticeable). Use of the integer data types: `intXX_t` and `uintXX_t` replacing the `XX` with the desired number of bits to match up to the equivalent number of bytes for the fields of the packet (ex. packet id = `uint16_t` = `2 bytes`).
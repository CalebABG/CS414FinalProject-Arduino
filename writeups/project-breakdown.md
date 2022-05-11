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

The data array for the packet is set to 16 bytes in the Arduino file defining the packet, but this can be easily changed; I’ve tested up to 64 bytes and the protocol held up fine. The communication of data via the Bluetooth module and the packet CRC were the trickiest parts. The processing of the data over Bluetooth is handled by a state-machine on the Arduino. If the bytes of the packet are in the correct sequence and the packet CRC is valid, the packet is processed, otherwise thrown out.


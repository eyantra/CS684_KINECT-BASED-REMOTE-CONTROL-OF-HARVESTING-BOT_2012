KINECT BASED REMOTE CONTROL OF HARVESTING BOT
---------------------------------------------------

Team
------
Ashok Nallagalva    123059014
Jyoti Gajrani       113053001
Pooja Mazumdar      113050001
Satish Vemireddy    123053001

Hardware Requirements
--------------------------
ï Kinect sensor device 
ï Two computer systems (one acts as server and other as client) 
ï Firebird V ATMEGA2560
ï X-bee for wireless communication between server system and bot 
ï IP camera for transmitting live video over internet to client

Software Requirements
--------------------------
ï The Kinect SDK beta is used to work on MS Windows 7 with DirectX SDK 
ï The Kinect Sensor Device requires a good processor and a dedicated VGA card 
ï X-CTU for con?guring X-bee 
ï IPCam Admin Utility for con?guring IP camera 
ï Microsoft Visual Studio 2010 for development of client and server applications in C# 
ï .NET framework 4.0 
ï AVR Studio for writing ?rebird code 
ï AVR bootloader for burning code to bot

Execution Instructions :
1. Do all the connections and configurations of ZigBee, IP Camera and Gripper arm as mentioned in hardware manual
2. Refer software manual to install required softwares to run this project
2. Burn the firebird code on firebird V (firebird/Serial_Communication_ZigBee_wireless/Serial_Communication_ZigBee_wireless/default/Serial_Communication_ZigBee_wireless.hex)
3. On server system, open ‚ÄúConsoleApplication1.sln‚Äù (in folder server_project/) file in visual studio and run the application
4. On client system, open ‚ÄúSkeletalViewer.sln‚Äù (in folder client_project/) file in visual studio and run the application
5. Provide proper gestures at client side (as demonstrated in the project video)

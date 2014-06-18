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
� Kinect sensor device 
� Two computer systems (one acts as server and other as client) 
� Firebird V ATMEGA2560
� X-bee for wireless communication between server system and bot 
� IP camera for transmitting live video over internet to client

Software Requirements
--------------------------
� The Kinect SDK beta is used to work on MS Windows 7 with DirectX SDK 
� The Kinect Sensor Device requires a good processor and a dedicated VGA card 
� X-CTU for con?guring X-bee 
� IPCam Admin Utility for con?guring IP camera 
� Microsoft Visual Studio 2010 for development of client and server applications in C# 
� .NET framework 4.0 
� AVR Studio for writing ?rebird code 
� AVR bootloader for burning code to bot

Execution Instructions :
1. Do all the connections and configurations of ZigBee, IP Camera and Gripper arm as mentioned in hardware manual
2. Refer software manual to install required softwares to run this project
2. Burn the firebird code on firebird V (firebird/Serial_Communication_ZigBee_wireless/Serial_Communication_ZigBee_wireless/default/Serial_Communication_ZigBee_wireless.hex)
3. On server system, open “ConsoleApplication1.sln” (in folder server_project/) file in visual studio and run the application
4. On client system, open “SkeletalViewer.sln” (in folder client_project/) file in visual studio and run the application
5. Provide proper gestures at client side (as demonstrated in the project video)

/////////////////////////////////////////////////////////////////////////
//   Project : Kinect based remote control of harvesting bot
//   File  : MainWindow.xaml.cs
//   Class : MainWindow
//   Description :
//              1. Processes Depth-data and Sekeltal-data read by 
//                 Kinect sensor
//              2. Updates Main window of the application
//              3. Recognizes gestures
//              4. Sends command to server using socket prorammin
//                
//   Date of Creation :     NA
//   Created by :           Microsoft
//   Date of Modification : Oct, 2012
//   Created by :           CS-684 Project Team-2
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
//
// This module contains code to do Kinect NUI initialization and
// processing and also to display NUI streams on screen.
//
// Copyright � Microsoft Corporation.  All rights reserved.  
// This code is licensed under the terms of the 
// Microsoft Kinect for Windows SDK (Beta) from Microsoft Research 
// License Agreement: http://research.microsoft.com/KinectSDK-ToU
//
/////////////////////////////////////////////////////////////////////////
using System;
using System.Threading;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Research.Kinect.Nui;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;

namespace SkeletalViewer
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constructors
        public MainWindow()
        {
            InitializeComponent();
            // TEAM 2_2012
            // Open TCP/IP socket
            tcpclnt.Connect("10.129.139.24", 8001);
            stm = tcpclnt.GetStream();
            myBrowser.Navigate(new Uri("http://10.129.139.3"));

              }
        #endregion

        #region Variables for initialization of Kinect
        Runtime nui;

        DateTime lastTime = DateTime.MaxValue;
        double rprevz;
        char s = '0';

        // We want to control how depth data gets converted into false-color data
        // for more intuitive visualization, so we keep 32-bit color frame buffer versions of
        // these, to be updated whenever we receive and process a 16-bit frame.
        const int RED_IDX = 2;
        const int GREEN_IDX = 1;
        const int BLUE_IDX = 0;
        byte[] depthFrame32 = new byte[320 * 240 * 4];
        // mapping the appropriate key words which are further used below

        // the map is obtained form kinect specs sheet
        Dictionary<JointID, Brush> jointColors = new Dictionary<JointID, Brush>() { 
            {JointID.HipCenter, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.Spine, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.ShoulderCenter, new SolidColorBrush(Color.FromRgb(168, 230, 29))},
            {JointID.Head, new SolidColorBrush(Color.FromRgb(200, 0,   0))},
            {JointID.ShoulderLeft, new SolidColorBrush(Color.FromRgb(79,  84,  33))},
            {JointID.ElbowLeft, new SolidColorBrush(Color.FromRgb(84,  33,  42))},
            {JointID.WristLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HandLeft, new SolidColorBrush(Color.FromRgb(215,  86, 0))},
            {JointID.ShoulderRight, new SolidColorBrush(Color.FromRgb(33,  79,  84))},
            {JointID.ElbowRight, new SolidColorBrush(Color.FromRgb(33,  33,  84))},
            {JointID.WristRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.HandRight, new SolidColorBrush(Color.FromRgb(37,   69, 243))},
            {JointID.HipLeft, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.KneeLeft, new SolidColorBrush(Color.FromRgb(69,  33,  84))},
            {JointID.AnkleLeft, new SolidColorBrush(Color.FromRgb(229, 170, 122))},
            {JointID.FootLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HipRight, new SolidColorBrush(Color.FromRgb(181, 165, 213))},
            {JointID.KneeRight, new SolidColorBrush(Color.FromRgb(71, 222,  76))},
            {JointID.AnkleRight, new SolidColorBrush(Color.FromRgb(245, 228, 156))},
            {JointID.FootRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))}
        };
        #endregion

        #region Our Variables
        //Struct for storing the x,y,z coordinates of a point
        public struct str
        {
            public double x, y, z;
        };
        #region Variables for serialport
        SerialPort serialPort;
       
        //Socket Programming Variables
        TcpClient tcpclnt = new TcpClient();
        Stream stm;
        ASCIIEncoding asen = new ASCIIEncoding();
        byte[] ba;

        //Temporary variables to store values to be written over serial port
        int send;
        int send2 = 0;
        #endregion

        //The current no. of frames
        int lastFrames = 0;
        int framediff = 30;
        //Total no. of frames after which data has to be sent
        int totalFrames = 10;
        //The previous value of angle detected in mode 1(Angle Rotation)
        double initangle = 80;
        //The previous value of angle detected in mode 1(Angle Rotation)
        double curangle = 0;
        int curx = 0;


        #region Variables for joint coordinates as returned by Kinect
        //r ---> RIGHT
        //l ---> LEFT
        public str r_wrist;   
        public str r_elbow;     
        public str r_shldr;
        public str r_hand;
        public str l_wrist;
        public str l_elbow;
        public str l_shldr;
        public str l_hand;
        public str shldr_center;
        public str hip_center;
        public str head;
        #endregion

        #region Variables for mode 2
        //The difference between (r/l)_wrist & previous z-coordinate of wrist(r/l) (to be sent)
        double rdiff = 0;
        double ldiff = 0;
        //The previous value of (r/l)_wrist
        public double rlast = 0;
        public double llast = 0;
        //Type casts of (r/l)diff to be written to serialport
        int rint = 0;
        int lint = 0;
        #endregion
        #region used in functions not called in project
        int fck = 10;
        int fcktemp = 0;
        #endregion
        //Threshold limit of deviation of angle received from kinect from required value
        double thresh = 40;
        //Variable for storing the current mode of gestures
        public int mode = 0;
        
        #region Variables for calculating vectors joining 2 points
        //vector joining right wrist and right elbow
        str r_we;
        //vector joining right elbow and right shoulder
        str r_es ;
        //vector joining right shoulder and right elbow
        str r_se ;
        //vector joining shoulder center and hip center
        str mid ;
        //vector joining right shoulder and left shoulder
        str pln ;
        //vector joining shoudler center and right wrist
        str r_wc;
        //vector joining right hand and right wrist
        str r_hw;
        //vector joining left shoulder and right shoulder
        str sl_sr;
        //vector joining left wrist and left elbow

        str l_we ;
        //vector joining left elbow and left shoulder
        str l_es ;
        //vector joining left shoulder and left elbow
        str l_se ;
        //vector joining shoulder center and left wrist
        str l_wc ;
        //vector joining left hand and left wrist
        str l_hw ;

        
        //coordinates of wrist with x coordinate replaced by that of shoulder x
        str r_pt;
        str l_pt;
        //Vector joinint (r/l)shoulder and (r/l)_pt
        str r_req ;
        str l_req ;
        #endregion

        #region Angles between two vectors
        //Angle between r_we,r_es
        double rx1 ;
        //Angle between r_es,mid
        double rx2 ;
        //Angle between r_se,r_we
        double rx3 ;
        //Angle between r_wc,r_pln
        double rx4 ;
        //Angle between r_wc,r_mid
        double rx5 ;
        //Angle between sl_sr,r_se
        double rx6 ;
        //Angle between r_we,mid
        double rx8 ;
        //Angle between r_we,pln
        double rx7 ;
        //Angle between r_es,pln
        double rx9 ;
        //Angle between r_req,mid
        double rx10;



        //Left hand analogues of the above
        double lx1;
        double lx2;
        double lx3;
        double lx4;
        double lx5;
        double lx6;
        double lx8;
        double lx7;
        double lx9;
        double lx10;
        #endregion


        // TEAM 2_2012
        // Variables for serial port communication.
        //SerialPort serialPort;

        // TEAM 2_2012
        // Variables to track the position of various joints and co-ordinates.

        #endregion

        private void Window_Loaded(object sender, EventArgs e)
        {
            // TEAM 2_2012
            // Initialization
            nui = Runtime.Kinects[0];

            try
            {
                nui.Initialize(RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor);
            }
            catch (InvalidOperationException)
            {
                System.Windows.MessageBox.Show("Runtime initialization failed. Please make sure Kinect device is plugged in.");
                return;
            }

            try
            {
                nui.VideoStream.Open(ImageStreamType.Video, 2, ImageResolution.Resolution640x480, ImageType.Color);
                nui.DepthStream.Open(ImageStreamType.Depth, 2, ImageResolution.Resolution320x240, ImageType.DepthAndPlayerIndex);

            }
            catch (InvalidOperationException)
            {
                System.Windows.MessageBox.Show("Failed to open stream. Please make sure to specify a supported image type and resolution.");
                return;
            }

            lastTime = DateTime.Now;

            // TEAM 2_2012
            // Setting function 'nui_DepthFrameReady' as event handler of depth frame. 
            // As a depth frames comes from Kinect to be processed, this function will be called.
            // Similarly setting function 'nui_SkeletonFrameReady' and
            // 'nui_ColorFrameReady' as event handlers.
            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
            nui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_ColorFrameReady);
        }

        //Function for displaying the depth frame on the screen
        // Converts a 16-bit grayscale depth frame which includes player indexes into a 32-bit frame
        // that displays different players in different colors
        byte[] convertDepthFrame(byte[] depthFrame16)
        {
            for (int i16 = 0, i32 = 0; i16 < depthFrame16.Length && i32 < depthFrame32.Length; i16 += 2, i32 += 4)
            {
                int player = depthFrame16[i16] & 0x07;
                int realDepth = (depthFrame16[i16 + 1] << 5) | (depthFrame16[i16] >> 3);
                // transform 13-bit depth information into an 8-bit intensity appropriate
                // for display (we disregard information in most significant bit)
                byte intensity = (byte)(255 - (255 * realDepth / 0x0fff));

                depthFrame32[i32 + RED_IDX] = 0;
                depthFrame32[i32 + GREEN_IDX] = 0;
                depthFrame32[i32 + BLUE_IDX] = 0;

                // choose different display colors based on player
                switch (player)
                {
                    case 0:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity / 2);
                        break;
                    case 1:
                        depthFrame32[i32 + RED_IDX] = intensity;
                        break;
                    case 2:
                        depthFrame32[i32 + GREEN_IDX] = intensity;
                        break;
                    case 3:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 4);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 4:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity / 4);
                        break;
                    case 5:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 4);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 6:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 7:
                        depthFrame32[i32 + RED_IDX] = (byte)(255 - intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(255 - intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(255 - intensity);
                        break;
                }
            }
            return depthFrame32;
        }

        void nui_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            PlanarImage Image = e.ImageFrame.Image;
            byte[] convertedDepthFrame = convertDepthFrame(Image.Bits);

            depth.Source = BitmapSource.Create(
                Image.Width, Image.Height, 96, 96, PixelFormats.Bgr32, null, convertedDepthFrame, Image.Width * 4);
             
            //calculating vectors as from the points as described in variables
            r_we = diff(r_wrist, r_elbow); // wrist elbow line
            r_es = diff(r_elbow, r_shldr); // elbow shoulder line
            r_se = diff(r_shldr, r_elbow); //  shoulder elbow line
            mid = diff(shldr_center, hip_center); //line parllel to man
            pln = diff(r_shldr, l_shldr); // line perpendicular to man
            r_wc = diff(shldr_center, r_wrist); 
            r_hw = diff(r_hand, r_wrist);
            sl_sr = diff(l_shldr, r_shldr);
            l_we = diff(l_wrist, l_elbow); //same counter parts above
            l_es = diff(l_elbow, l_shldr);//same counter parts above
            l_se = diff(l_shldr, l_elbow);//same counter parts above
            l_wc = diff(shldr_center, l_wrist);//same counter parts above
            l_hw = diff(l_hand, l_wrist);

            r_pt = r_wrist;
            r_pt.x = r_shldr.x;
            l_pt = l_wrist;
            l_pt.x = l_shldr.x;
            r_req = diff(r_shldr, r_pt);
            l_req = diff(l_shldr, l_pt);

            // angles between the lines
            rx1 = angle(r_we, r_es); //  wrist-elbow elbow-shoulder angle
            rx2 = angle(r_es, mid); // elbow shoulder  elevation

            rx3 = angle(r_se, r_we); // reverse of rx1
         
            rx6 = angle(sl_sr, r_se);
            rx8 = angle(r_we, mid); // wrist-elbow  elevation
            rx7 = angle(r_we, pln); // writs-elbow with horizontal
            rx9 = angle(r_es, pln); // elbow-shoulder with horizontal
            rx10 = angle(r_req, mid); //mapping used for control of gripper arm

            lx1 = angle(l_we, l_es); //the left counter parts of above
            lx2 = angle(l_es, mid);
            lx3 = angle(l_se, l_we);
            lx4 = angle(l_wc, pln);
            lx5 = angle(l_wc, mid); // the left counter parts of above
            lx6 = angle(sl_sr, l_se);
            lx8 = angle(l_we, mid);
            lx7 = angle(l_we, pln);
            lx9 = angle(l_es, pln);
            lx10 = angle(l_req, mid);

            #region debug messages
            //this.semaphore1.Text = "rx 2,9 :" + rx2 + " " + rx9 + " ";
            //this.semaphore2.Text = "rx 7,3 : " + rx7 + " " + rx3;
            //this.semaphore3.Text = "lx 2,9 :" + lx2 + " " + lx9;
            //this.semaphore4.Text = "lx 7,3 : " + lx7 + " " + lx3;
            //this.semaphore5.Text = "rx 8,lx 8 : " + rx8 + " " + lx8;
            #endregion 
            //DECIDING THE MODE

            if (check(rx2 - 90, rx9, rx7 - 90, rx8) == 1)
            {
                //WHEN BOTH LEFT AND RIGHT HANDS ARE IN POSITION AS REQUIRED (GRIPPER MODE)
                mode = 3;
                lastFrames = 0;
                char x = (char)mode;
                ba = asen.GetBytes(x.ToString());
                stm.Write(ba, 0, ba.Length);
              //  this.serialPort.Write(x.ToString());
            }
            else if (check(rx2 - 90, rx9 - 90, rx7 - 90, rx8) == 1)
            {
                //WHEN RIGHT HAND IS IN POSITION (LONG RANGE MODE)
                mode = 1;
                lastFrames = 0;
                char x = (char)mode;
                ba = asen.GetBytes(x.ToString());
                stm.Write(ba, 0, ba.Length);
             //   this.serialPort.Write(x.ToString());
            }
            else if (check(lx2 - 90, lx9 - 90, lx7 - 90, lx8) == 1)
            {
                //WHEN LEFT HAND IS IN POSTION (FINE CONTROL MODE)
                mode = 2;
                //Introducing delay
                for (int i = 0; i < 1000000; i++)
                {
                    mode=1+1;
                }
                lastFrames = 0;
                
                //setting (r/l)last to the value of (r/l)wrist
                rlast = r_wrist.z;
                llast = l_wrist.z;

                char x = (char)mode;
                ba = asen.GetBytes(x.ToString());
                stm.Write(ba, 0, ba.Length);
              //  this.serialPort.Write(x.ToString());
            }
            else
            {
               
                ;
                
            }
            this.semaphore.Text = "MODE : " + mode;
            //WHEN IN MODE 1 i.e. long range motion (* put if condition)

            if (mode == 1)
            {
                //FOR FORWARD MOTION
                if (check(rx2 - 90, rx9 - 90, rx7 - 90, rx8 - 90) == 1 && check(lx2 - 90, lx9 - 90, lx7 - 90, lx8 - 90) == 1)
                {
                    //COMMUNICATE SIGNAL CORRESPONDING TO THIS
                    this.semaphore.Text = "IN MODE 1 FORWARD MOTION DETECTED";
                    char x = (char)8;
                    ba = asen.GetBytes(x.ToString());
                    stm.Write(ba, 0, ba.Length);

                    lastFrames = 0;
                }

                //FOR STOP
                else if (check(rx2, rx9 - 90, rx7 - 90, rx8) == 1)
                {
                    //STOP
                    this.semaphore.Text = "STOP";
                    char x = (char)5;
                    ba = asen.GetBytes(x.ToString());
                    stm.Write(ba, 0, ba.Length);

                    lastFrames = 0;
                }

                else if (check(rx2 - 180, rx9 - 90, rx3 - 90, rx7 - 90) == 1 && check(lx2 - 180, lx9 - 90, lx3 - 90, lx7 - 90) == 1)
                {
                    //BACK
                    this.semaphore.Text = "BACK";
                    char x = (char)9;
                    ba = asen.GetBytes(x.ToString());
                    stm.Write(ba, 0, ba.Length);

                    lastFrames = 0;
                }
                //ANGLE ROTATION
                else if (check(rx2 - 90, 0, 0, rx8 - 90) == 1)
                {
                    //SEND ANGLE BETWEEN HAND AND PLN
                    curangle = rx9;
                    if (lastFrames == 0)
                    {
                        //If fresh segment(set of frames to be sampled) change the value of previous angle stored to current angle
                        initangle = curangle;
                    }
                    lastFrames++;

                    this.semaphore.Text = "IN MODE 1 ANGLE DETECTION : " + send + " s : " + s.ToString();
                    //When no. of frames exceeds the sample size
                    if (lastFrames >= totalFrames)
                    {
                        send = -(Convert.ToInt32(curangle) - Convert.ToInt32(initangle));
                        //Nullifying minor changes
                        if (send < 10 && send > -10)
                        {
                            send = 0;
                        }
                        if (send < 0)
                        {
                            send = 45 - send;
                        }
                        //Adding 10 to the value to be sent
                        send += 10;
                       
                        s = (char)send;
                        ba = asen.GetBytes(s.ToString());
                        stm.Write(ba, 0, ba.Length);
                   //     this.serialPort.Write(s.ToString());
                        lastFrames = 0;
                        initangle = curangle;
                    }

                }
                else
                {
                    //stop condition
                    lastFrames = 0;
          /*          s = (char)5;
                    ba = asen.GetBytes(s.ToString());
                    stm.Write(ba, 0, ba.Length);
                
            */        
                }
            }

            //WHEN IN MODE 2
            else if (mode == 2)
            {

                if (check(lx2 - 90, 0, 0, lx8 - 90) == 1)
                {
                    //SEND ANGLE BETWEEN HAND AND PLN
                    curangle = lx9;
                    if (lastFrames == 0)
                    {
                        //If fresh segment(set of frames to be sampled) change the value of previous angle stored to current angle
                        initangle = curangle;
                    }
                    lastFrames++;

                    this.semaphore.Text = "IN MODE 2 ANGLE DETECTION : " + send + " s : " + s.ToString();
                    //When no. of frames exceeds the sample size
                    if (lastFrames >= totalFrames)
                    {
                        send = -(Convert.ToInt32(curangle) - Convert.ToInt32(initangle));
                        //Nullifying minor changes
                        if (send < 10 && send > -10)
                        {
                            send = 0;
                        }
                        if (send < 0)
                        {
                            send = 45 - send;
                        }
                        //Adding 10 to the value to be sent
                        send += 10;

                        s = (char)send;
                        ba = asen.GetBytes(s.ToString());
                        stm.Write(ba, 0, ba.Length);
                        //     this.serialPort.Write(s.ToString());
                        lastFrames = 0;
                        initangle = curangle;
                    }

                }
                else
                {
                    lastFrames = 0;

                }
            }

            //WHEN IN MODE 3
            else if (mode == 3)
            {
                if (check(rx3 - 180, rx9 - 90, 0, rx6 - 90) == 1)
                {
                    lastFrames++;
                    //RETURN rx10,lx10;
                    //RETURN rx9,lx9
                   // sampling for ten frammes
                    if (lastFrames >= totalFrames)
                    {
                        lastFrames = 0;
                        //Maximum positive angle that can be sent is 90
                        if (rx10 > 90) rx10 = 90;
                        if (rx10 < 0) rx10 = 0;
                        rx10 = Math.Abs(rx10);

                        //sleep for synchronization
                        Thread.Sleep(100);
                        
                        //Scaling the angle appropriately
                        try
                        {
                            send = Convert.ToInt32(90-rx10);   // angle range from 0 to 90 degree
                            //send = Convert.ToInt32(rx10 * (-11.0 / 18.0) + 70); // range of angle 15 - 70 degree
                        }
                        catch
                        {
                            this.semaphore.Text = "Angle scaling is giving value larger than Int32";
                        }
                       
                        char x = (char)send;
                        ba = asen.GetBytes(x.ToString());
                        stm.Write(ba, 0, ba.Length);

                        if (check(lx2 - 90, lx6 - 180, 0,0) == 1)
                        {      //Maximum positive angle that can be sent is 90
                            if (lx1 > 90) lx1 = 90;
                            if (lx1 < 0) lx1 = 0;
                            //Scaling the angle appropriately

                            send2 = Convert.ToInt32(lx1 * (1.0 / 3));
                            send2 += 90;
                            Thread.Sleep(100);
                            x = (char)send2;
                            ba = asen.GetBytes(x.ToString());
                            stm.Write(ba, 0, ba.Length);
                            //        this.serialPort.Write(x.ToString());
                        }
                    }
                    this.semaphore.Text = "IN MODE 3 : lx9, rx10 " + lx9 + " " + rx10 + " " + send;
                }
                
            }
            else
            {
                ;
            }
        }

        //angles constant ew,bodyline 90 ; we,sline 90 /
        //returns +1 if positive motion -1 if -ve motion 0 if no motion 2 if future possibility of motion


        // checks if 3 angles are less around 0 -- returns 1 if true and 0 if false
        public int check(double a1, double a2, double a3, double a4)
        {
            if (Math.Abs(a1) < thresh && Math.Abs(a2) < thresh && Math.Abs(a3) < thresh && Math.Abs(a4) < thresh)
                return 1;
            return 0;
        }

        
  
        //Calculates the difference between two coordinates and returns the result
        //used for creating a new vector from existing two vectors
        public str diff(str x1, str x2)
        {
            str res;
            res.x = x1.x - x2.x;
            res.y = x1.y - x2.y;
            res.z = x1.z - x2.z;
            return res;
        }

        //Calculates the angle between two  vectors using dot product and returns the result
        public double angle(str x1, str x2)
        {
            double sum = x1.x * x2.x + x1.y * x2.y + x1.z * x2.z;
            double n1 = Math.Sqrt(x1.x * x1.x + x1.y * x1.y + x1.z * x1.z);
            double n2 = Math.Sqrt(x2.x * x2.x + x2.y * x2.y + x2.z * x2.z);
            sum = Math.Acos(sum / (n1 * n2)) * 180 / Math.PI;
            return sum;

        }

       
        // getting the current frame from kinect
        private Point getDisplayPosition(Joint joint)
        {
            float depthX, depthY;
            nui.SkeletonEngine.SkeletonToDepthImage(joint.Position, out depthX, out depthY);
            depthX = depthX * 320; //convert to 320, 240 space
            depthY = depthY * 240; //convert to 320, 240 space
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();
            // only ImageResolution.Resolution640x480 is supported at this point
            nui.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // map back to skeleton.Width & skeleton.Height
            return new Point((int)(skeleton.Width * colorX / 640.0), (int)(skeleton.Height * colorY / 480));
        }

        //default function for getting intial data
        Polyline getBodySegment(Microsoft.Research.Kinect.Nui.JointsCollection joints, Brush brush, params JointID[] ids)
        {
            PointCollection points = new PointCollection(ids.Length);
            for (int i = 0; i < ids.Length; ++i)
            {
                points.Add(getDisplayPosition(joints[ids[i]]));
            }

            Polyline polyline = new Polyline();
            polyline.Points = points;
            polyline.Stroke = brush;
            polyline.StrokeThickness = 5;
            return polyline;
        }

        // getting data from camera and creating the appropriate structures
        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame skeletonFrame = e.SkeletonFrame;
            int iSkeleton = 0;
            Brush[] brushes = new Brush[6];
            brushes[0] = new SolidColorBrush(Color.FromRgb(255, 0, 0));
            brushes[1] = new SolidColorBrush(Color.FromRgb(0, 255, 0));
            brushes[2] = new SolidColorBrush(Color.FromRgb(64, 255, 255));
            brushes[3] = new SolidColorBrush(Color.FromRgb(255, 255, 64));
            brushes[4] = new SolidColorBrush(Color.FromRgb(255, 64, 255));
            brushes[5] = new SolidColorBrush(Color.FromRgb(128, 128, 255));

            skeleton.Children.Clear();
            foreach (SkeletonData data in skeletonFrame.Skeletons)
            {
                if (SkeletonTrackingState.Tracked == data.TrackingState)
                {
                    // Draw bones
                    Brush brush = brushes[iSkeleton % brushes.Length];
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.Spine, JointID.ShoulderCenter, JointID.Head));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderLeft, JointID.ElbowLeft, JointID.WristLeft, JointID.HandLeft));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderRight, JointID.ElbowRight, JointID.WristRight, JointID.HandRight));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipLeft, JointID.KneeLeft, JointID.AnkleLeft, JointID.FootLeft));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipRight, JointID.KneeRight, JointID.AnkleRight, JointID.FootRight));

                    // Draw joints
                    //motion = 0;
                    foreach (Joint joint in data.Joints)
                    {
                        Point jointPos = getDisplayPosition(joint);
                        Line jointLine = new Line();
                        jointLine.X1 = jointPos.X - 3;
                        jointLine.X2 = jointLine.X1 + 6;

                        jointLine.Y1 = jointLine.Y2 = jointPos.Y;
                        jointLine.Stroke = jointColors[joint.ID];
                        jointLine.StrokeThickness = 6;
                        skeleton.Children.Add(jointLine);

                        // TEAM 2_2012
                        // POPULATE DATA
                        // Here populate programming variables with the data processed 
                        if (joint.ID == JointID.WristRight)
                        {
                            r_wrist.x = joint.Position.X;
                            r_wrist.y = joint.Position.Y;
                            r_wrist.z = joint.Position.Z;

                        }
                        //getting the elbow co-ordinates
                        if (joint.ID == JointID.ElbowRight)
                        {
                            r_elbow.x = joint.Position.X;
                            r_elbow.y = joint.Position.Y;
                            r_elbow.z = joint.Position.Z;

                        }
                        //getting shoulder co-ordinates
                        if (joint.ID == JointID.ShoulderRight)
                        {
                            r_shldr.x = joint.Position.X;
                            r_shldr.y = joint.Position.Y;
                            r_shldr.z = joint.Position.Z;

                        }
                        // same as above the item can be known from templates
                        if (joint.ID == JointID.HandRight)
                        {
                            r_hand.x = joint.Position.X;
                            r_hand.y = joint.Position.Y;
                            r_hand.z = joint.Position.Z;
                        }
                        if (joint.ID == JointID.ShoulderLeft)
                        {
                            l_shldr.x = joint.Position.X;
                            l_shldr.y = joint.Position.Y;
                            l_shldr.z = joint.Position.Z;
                        }
                        if (joint.ID == JointID.WristLeft)
                        {
                            l_wrist.x = joint.Position.X;
                            l_wrist.y = joint.Position.Y;
                            l_wrist.z = joint.Position.Z;

                        }
                        if (joint.ID == JointID.ElbowLeft)
                        {
                            l_elbow.x = joint.Position.X;
                            l_elbow.y = joint.Position.Y;
                            l_elbow.z = joint.Position.Z;

                        }
                        if (joint.ID == JointID.HandLeft)
                        {
                            l_hand.x = joint.Position.X;
                            l_hand.y = joint.Position.Y;
                            l_hand.z = joint.Position.Z;
                        }
                        if (joint.ID == JointID.ShoulderCenter)
                        {
                            shldr_center.x = joint.Position.X;
                            shldr_center.y = joint.Position.Y;
                            shldr_center.z = joint.Position.Z;

                        }

                        if (joint.ID == JointID.HipCenter)
                        {
                            hip_center.x = joint.Position.X;
                            hip_center.y = joint.Position.Y;
                            hip_center.z = joint.Position.Z;
                        }
                    }
                }
                iSkeleton++;
            } // for each skeleton
        }
        // showing the actual video to the user
        void nui_ColorFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            // 32-bit per pixel, RGBA image
            PlanarImage Image = e.ImageFrame.Image;
            video.Source = BitmapSource.Create(
                Image.Width, Image.Height, 96, 96, PixelFormats.Bgr32, null, Image.Bits, Image.Width * Image.BytesPerPixel);
        }
        //exiting function
        private void Window_Closed(object sender, EventArgs e)
        {
            nui.Uninitialize();
            // Close the tcp socket
            ba = asen.GetBytes("End");
            stm.Write(ba, 0, ba.Length);
            tcpclnt.Close();

            Environment.Exit(0);
        }

    }
}

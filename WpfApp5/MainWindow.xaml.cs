using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using Microsoft.Kinect;
using Microsoft.Kinect.VisualGestureBuilder;
using System.Runtime.InteropServices;
using System.Collections;
using System.Diagnostics;
using System.Windows.Forms;

namespace WpfApp5
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinectSensor = null;

        private BodyFrameReader bodyFrameReader = null;

        //True when switching hand to control
        private static bool resetOrigPos = false;

        //0 represent right hand, 1 represent left hand
        private static int whichhand = 1;

        //Store the current trackingId
        private static ulong currentbodyId = 100;
        
        //body array for storing the bodies from each frame arrived
        private Body[] bodies = null;

        //check if there was a click 
        private bool clicked = false;

        //Set time inverval limit between each click
        private Stopwatch stopwatch;

        //The count of the queue that smoothen the cursor
        private static int threshold = 8;

        //The joint queue that smoothen the cursor movement
        private static Queue JointQueue = new Queue();

        //Get the screen height
        private static readonly int screenHeight = (int)System.Windows.SystemParameters.PrimaryScreenHeight;
        //Get the screen width
        private static readonly int screenWidth = (int)System.Windows.SystemParameters.PrimaryScreenWidth;

        //Get the original hand position to adjust the cursor position based on where users wants their hands to be.
        CameraSpacePoint OriginalHandPos = new CameraSpacePoint();

        //Sensitivity
        private static int sensitivity = 1700;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();

            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            this.kinectSensor.Open();

            this.bodyFrameReader.FrameArrived += this.Reader_frameArrived;

            this.stopwatch = new Stopwatch();
        }
        //DLLImport

        //SetCursorPos
        [DllImport("user32.dll")]
        private static extern bool SetCursorPos(int x, int y);
        //GetCursorPos
        [DllImport("user32.dll")]
        private static extern bool GetCursorPos(out Point lpPoint);
        //Mouse Event
        [DllImport("user32.dll")]
        private static extern void mouse_event(int dwFlags, int dx, int dy, int cButtons, int dwExtraInfo);
        //Flags
        private const int MOUSEEVENTF_LEFTDOWN = 0x02;
        private const int MOUSEEVENTF_LEFTUP = 0x04;
        private const int MOUSEEVENTF_RIGHTDOWN = 0x08;
        private const int MOUSEEVENTF_RIGHTUP = 0x10;
        private const int MOUSEEVENTF_WHEEL = 0x0800;

        //FrameReader that handles frame_arrive event
        private void Reader_frameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        //initiate the body array
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }
                    //update the body array
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
                if (dataReceived)
                {
                    if (this.bodies != null)
                    {
                        //If the currentbodyID is not 0, and the previous body is still being tracked
                        if (currentbodyId != 0 &&If_bodystillTracked())
                        {
                            foreach (Body body in bodies)
                            {
                                //if that is the body to be tracked
                                if (body.TrackingId == currentbodyId)
                                {
                                    //get the joints
                                    Joint righthand = body.Joints[JointType.HandRight];
                                    Joint lefthand = body.Joints[JointType.HandLeft];
                                    Joint spinemid = body.Joints[JointType.SpineMid];
                                    Joint spinebase = body.Joints[JointType.SpineBase];
                                    //if right hand is controlling the cursor
                                    if (if_righthand(lefthand, righthand, spinemid, spinebase))
                                    {
                                        //if it was not right hand controlling, that is whichhand !=0, change to righthand -- whichhand = 0, and
                                        //set resetOrigPos to true
                                        //whichhand = 0 --> represent righthand
                                        if (whichhand != 0)
                                        {
                                            whichhand = 0;
                                            //if resetOrigPos is true, clean the Joint Array to reset OriginalPosition
                                            resetOrigPos = true;
                                        }
                                        //call the clicking method
                                        doClick(righthand, body);
                                    }
                                    //if lefthand should be tracked
                                    if (if_lefthand(lefthand, righthand, spinemid, spinebase))
                                    {
                                        //whichand = 1 --> lefthand
                                        if (whichhand != 1)
                                        {
                                            //set controlling hand to left
                                            whichhand = 1;
                                            //resetOriginPos to true;
                                            resetOrigPos = true;
                                        }
                                        //call clicking method
                                        doClick(lefthand, body);
                                    }
                                }
                            }
                        }
                        //if the previous-frame-tracking body is lost, find the closest body and update the currentbodyId
                        else 
                        {
                            currentbodyId = GetRightBody().TrackingId;
                        }
                    }                    
                }
            }
        }

        //The click method
        private void doClick(Joint hand, Body trackbody)
        {
            //If resetOrigPos is true, clean the Joint array and set it to false;
            if (resetOrigPos)
            {
                JointQueue.Clear();
                resetOrigPos = false;
            }
            //sumX represents the sum of Joints' x value in the array
            int sumX = 0;
            //sumY represents the sum of Joints' y value in the array
            int sumY = 0;
            //PreviousJoint from the last frame
            Joint previousJoint = new Joint();
            //This is for the startingof the Joint queue, since there is nothing to peek
            if (JointQueue.Count == 0) JointQueue.Enqueue(hand);
            else
            {
                //if there is something to peek, let the previousjoint equal to it
                previousJoint = (Joint)JointQueue.Peek(); 
                //if the move by user is too small, don't pass in the new value-- it can be trembling
                if ((Math.Abs(previousJoint.Position.X- hand.Position.X) >0.002 && Math.Abs(previousJoint.Position.Y- hand.Position.Y) > 0.002) &&
                        (Math.Abs(previousJoint.Position.X - hand.Position.X) < 0.5 && Math.Abs(previousJoint.Position.Y - hand.Position.Y) < 0.5))
                {
                    //if not, enqueue the new joint
                    JointQueue.Enqueue(hand);
                }
            }
            //get the original hand position to make sure whereever your hand is, it is centerd
            if (JointQueue.Count == 1)
            {
                OriginalHandPos = ((Joint)JointQueue.Peek()).Position;
            }
            //if the joint queue is larger than threshold, dequeue
            if (JointQueue.Count >= threshold)
            {
                JointQueue.Dequeue();
            }
            //limit the originalhand position to be in Absof(1)
            if (Math.Abs(OriginalHandPos.X) > 1)
            {
                OriginalHandPos.X = 0;
            }
            if (Math.Abs(OriginalHandPos.Y) > 1)
            {
                OriginalHandPos.Y = 0;
            }
            foreach (Joint jt in JointQueue)
            {
                //get the joint positions
                CameraSpacePoint cameraSpacePoint = jt.Position;
                //get the center point of the screen
                int ScreenMidX = screenWidth / 2;
                int ScreenMidY = screenHeight / 2;
                //change the real-world location to screen 
                float PosX = ScreenMidX + (cameraSpacePoint.X - OriginalHandPos.X) * (screenWidth+sensitivity);
                float PosY = ScreenMidY - (cameraSpacePoint.Y - OriginalHandPos.Y) * (screenHeight+sensitivity);
                sumX += (int)PosX ;
                sumY += (int)PosY ;

            }
            int count = JointQueue.Count;
            //Set the cursor position
            SetCursorPos(sumX / count, sumY / count);
            //if the queue is not empty
            if (count > 0)
            {
                //if the current tracking hand is right
                if (whichhand ==0)
                {
                    //if righthand is closed
                    if (trackbody.HandRightState == HandState.Closed)
                    {
                        //if the close hand moves more than a threshold, it is a scroll
                        if (Math.Abs(previousJoint.Position.Y - hand.Position.Y) > 0.007 )
                        {
                            mouse_event(MOUSEEVENTF_WHEEL, 0, 0, (int)((previousJoint.Position.Y - hand.Position.Y) * 1000), 0);
                        }
                        //else, it is a click
                        else
                        {
                            //if the interval between last click is greater than 500 or this is first time 
                            if (stopwatch.ElapsedMilliseconds == 0 || stopwatch.ElapsedMilliseconds > 500)
                            {
                                mouse_event(MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, sumX / JointQueue.Count, sumY / JointQueue.Count, 0, 0);
                                //restart the stopwatch
                                stopwatch.Restart();
                            }
                        }
                    }
                }
                else
                {
                    if (trackbody.HandLeftState == HandState.Closed)
                    {
                        if (Math.Abs(previousJoint.Position.Y - hand.Position.Y) > 0.01)
                        {
                            mouse_event(MOUSEEVENTF_WHEEL, 0, 0, (int)((previousJoint.Position.Y - hand.Position.Y) * 1000), 0);
                        }
                        else
                        {
                            if (!clicked)
                            {
                                mouse_event(MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, sumX / JointQueue.Count, sumY / JointQueue.Count, 0, 0);
                                clicked = true;
                            }
                        }
                    }
                }     
            }
        }
        //check if the user is using lefthand to control
        private bool if_lefthand(Joint left, Joint right, Joint spinemid, Joint spinebase)
        {
            if (left.Position.Z < spinemid.Position.Z && left.Position.Z < right.Position.Z && left.Position.Y > spinebase.Position.Y)
            {
                return true;
            }
            return false;
        }
        //check if the user is using righthand to control
        private bool if_righthand(Joint left, Joint right, Joint spinemid, Joint spinebase)
        {
            if (right.Position.Z < spinemid.Position.Z && left.Position.Z > right.Position.Z && right.Position.Y > spinebase.Position.Y)
            {
                return true;
            }
            return false;
        }
        //get the closest body in the frame
        private Body GetRightBody()
        {
            int index = 0;
            float smallest = 100;

            for (int i =0; i < bodies.Length; i++)
            {
                if (bodies[i].Joints[JointType.SpineMid].Position.Z <= smallest && bodies[i] != null && bodies[i].IsTracked)
                {
                    smallest = bodies[i].Joints[JointType.SpineMid].Position.Z;
                    index = i;  
                }
            }
            return bodies[index];
        }
        //check if the current body is still being tracked-- that is , the trackingid is not lost
        private bool If_bodystillTracked()
        {
            foreach (Body body in bodies)
            {
                if (body.TrackingId == currentbodyId)
                {
                    
                    return true;
                }
            }
            return false;
        }
    }
}

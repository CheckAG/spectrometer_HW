using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;

namespace SerialSpeedTester
{
    public partial class Form1 : Form
    {
        //Serial port-related variables
        private SerialPort MCU_serialport; //serial port instance 
        private bool isPortOpened = false;

        //Buffer-related variables
        private List<byte[]> incomingData = new List<byte[]>();
        private List<int> fileBuffer = new List<int>(); //buffer for writing into file
        private byte[] serialBuffer; //buffer for capturing the serial port datastream
        private byte[] bufferArray; //main buffer that stores the flattened serialBuffer
        private int totalBufferLength = 0; //length of the total captured AND converted 24-bit numbers
        private int ConvertedNumber; //Final result which is our 24-bit number. This gets stored in the fileBuffer, line by line

        //String-related variables
        private string incomingString = ""; //the total incoming data as a string
        private List<string> incomingStringBuffer = new List<string>();

        string filepath = String.Format(@"{0}\", Application.StartupPath); //puts the file in the same folder as the exe file

        //Timing related variables for managing the buffers
        private DateTime dt1, dt2; //dt1/2 are the variables which serve the purpose as timers for the buffers
        private double elapsedSeconds; //the elapsed time in seconds is stored in this variable

        public Form1()
        {
            InitializeComponent();
        }

        private void comPortComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            MCU_serialport = new SerialPort(); //create new instance
            MCU_serialport.BaudRate = 2000000; //might be higher in the future
            MCU_serialport.PortName = comPortComboBox.Text;
            MCU_serialport.Parity = Parity.None;
            MCU_serialport.DataBits = 8;
            MCU_serialport.StopBits = StopBits.One;
            MCU_serialport.NewLine = "\n"; //could be "\r\n" also?

            try
            {
                MCU_serialport.Open(); //try to open the port
                isPortOpened = true; //change the value of the variable

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "Error!"); //Something is wrong
            }
        }

        private void sendButton_Click(object sender, EventArgs e)
        {
            if (isPortOpened) //if the port is opened
            {
                try
                {
                    MCU_serialport.WriteLine(sendTextBox.Text.ToString()); //sends the content of the send textbox
                    if (binaryInputCheckBox.Checked)
                    {
                        serialCheckTimer.Start(); //start polling the serial port once we sent out the command
                        printTimer.Start(); //start updating the terminal with a certain frequency (timer intervall)
                    }
                    else
                    {
                        serialStringTimer.Start(); //start polling the serial port for reading strings
                    }
                    dt1 = DateTime.Now; //"note down" the time now, so dt2 can be compared to the starting time
                }
                catch
                { 
                    //not implemented
                }
            }
            else
            {
                MessageBox.Show("You haven't opened the serial port!"); //tell the user that he forgot to open the serial port
            }
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (isPortOpened) //If the port is opened
            {
                try
                {
                    MCU_serialport.Close();
                }
                catch
                {
                    //not implemented
                }
            }
            else
            {
                //do nothing as there is not 
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            getAvailablePorts(); //fetch the ports if there are any
        }


        void getAvailablePorts() // get the available ports and list them to the combobox
        {
            String[] ports = SerialPort.GetPortNames(); //fill up the ports array
            comPortComboBox.Items.AddRange(ports);
        }

        private void serialCheckTimer_Tick(object sender, EventArgs e)
        {
            //this timer supposed to create ticks that checks the buffer on the serial port and grabs it if there's something available
            //So, the timer is technically polling the serial port at every tick interval (what is a good time? 10 ms, 100 ms?)

            int bytesToRead = 0; //This variable stores the bytes to read. we can have max 2^32 bytes - 4 gigabytes

            serialCheckTimer.Stop(); //while we process the data, we are not supposed to run the clock while doing the buffering

            try
            {
                bytesToRead = MCU_serialport.BytesToRead; //tries to check the amount of bytes in the serial buffer
            }
            catch 
            {
            //not implemented yet(?)
            }

            if (MCU_serialport.IsOpen) //if the serial port is open (selected in the drop-down list)
            {
                if (bytesToRead != 0) //if there is something waiting on the serial port buffer
                {
                    //Since serialBuffer is always a new instance, it starts from "scratch", it is empty every time the code reaches this point
                    serialBuffer = new byte[bytesToRead]; //creates an array called serialBuffer that stores the bytes to read                   

                    if (MCU_serialport.IsOpen) //maybe this condition can be deleted(?)
                    {
                        MCU_serialport.Read(serialBuffer, 0, bytesToRead); //serialBuffer[bytesToRead] 
                    }

                    //incoming data is just being expanded with every tick reading, it will be copied and emptied somewhere else
                    incomingData.Add(serialBuffer); //now this becomes a matrix: incomigData[serialBuffer[bytesToRead]]

                    totalBufferLength += bytesToRead; //adds the bytes to read to the total buffer length
                    totalBufferSizeLabel.Text = totalBufferLength.ToString(); //this shows how many bytes are read from the serial port

                    //The invoming data is always 24 bits or 3 bytes, so the data shoud be grouped into 3 bytes then converted into decimal
                    //The incoming data is always the serialBuffer variable which holds all the bytes we just recently fetched from the serial buffer

                    /* --temporarily commented, but needed for debugging
                    foreach (int number in serialBuffer)
                    {                        
                        //This part converts the content of the serialBuffer(!, 3 bytes) into something readable
                        receiveTextBox.AppendText(number.ToString() + Environment.NewLine); //Appendtext automatically scrolls to the bottom of the text box   
                    }
                    */

                }
                serialCheckTimer.Start(); //restarts the timer after we finished - ticking can be continued
            }

        }

        private void printTimer_Tick(object sender, EventArgs e)
        {
            //this timer should update the terminal at every tick. The tick should be synced to the speed
            //receiveTextBox.AppendText("Dump Buffer: " + Environment.NewLine);

            bufferArray = new byte[totalBufferLength]; //by instantiating a new array, this buffer is always fresh (empty)

            bufferArray = incomingData.SelectMany(a => a).ToArray(); //flatten the matrix into a vector (array) and pass it to a new array
            incomingData.Clear(); //incomingData is cleared out here after passing its values to the other buffer

            int bufferArrayLength = bufferArray.Length; //length of the converted array (total number of bytes)
            int bufferArrayNumbers = bufferArrayLength / 3; //number of numbers (1 number is 3 byte or 24 bit, so we have to divide the array length by 3)
            buffersizeLabel.Text = bufferArrayNumbers.ToString(); //shows the buffer size (how many numbers are stored in a tick)

            /* --temporarily commented, but needed for debugging
            //receiveTextBox.AppendText("Array Length: " + bufferArrayLength.ToString() + " Numbers: " + bufferArrayNumbers.ToString() + Environment.NewLine);
            foreach (int numbers in bufferArray) //this serves only a test purpose, it prints all the 3 components of the 3-byte number
            {
                receiveTextBox.AppendText(numbers.ToString() + Environment.NewLine); //Appendtext automatically scrolls to the bottom of the text box  
            }
            */

            //we have to recreate the original numbers (012)(345)(678)...
            for (int i = 0; i < bufferArrayNumbers; i++) //this for() iterates through the number of 24-bit numbers
            {
                ConvertedNumber = 0; //converted number which is the incoming 3 bytes put into a 32-bit integer
                
                //MSB - 24 comes in:
                ConvertedNumber = bufferArray[((3 * i) + 2)] << 16; //23-16 bit is shifted to its place
                //receiveTextBox.AppendText("Step 1: " + ConvertedNumber.ToString() + Environment.NewLine);                
                ConvertedNumber |= bufferArray[((3 * i) + 1)] << 8; //15-8 bit is shifted to its place and combined with the previous number using bitwise OR
                //receiveTextBox.AppendText("Step 2: " + ConvertedNumber.ToString() + Environment.NewLine);
                ConvertedNumber |= bufferArray[3 * i]; //7-0 bit is combined with the previous 2 numbers using the bitwise OR
                //receiveTextBox.AppendText("Converted number: " + ConvertedNumber.ToString() + Environment.NewLine);

                //maybe here we can add another array or list and store some data, then dump it into a txt file with another timer ticks (elapsed seconds....etc)
                //Strategy would be the same buffering: collect X number of data, dump into a file, clear, collect, dump (append), clear...repeat...
                fileBuffer.Add(ConvertedNumber); //putting every new numbers in the fileBuffer

                if (enablePrintCheckBox.Checked) //if we want, we can print the final numbers on the terminal - at high speeds, this can make it to freeze up
                {
                    receiveTextBox.AppendText("Converted number: " + ConvertedNumber.ToString() + Environment.NewLine);
                }
            }

            dt2 = DateTime.Now; //check the time now - this (dt2) is compared to dt1 in the next steps
            elapsedSeconds = ((TimeSpan)(dt2 - dt1)).TotalSeconds; //calculate the difference between the start time and the currently checked time

            if (elapsedSeconds > 3) //for now, the buffer holds the data for 3 seconds
            {
                //print to file
                printToFile(); // print (append) to file                

                //reset timer - 3 second counting starts over
                dt1 = DateTime.Now;                
            }
        }

        private void serialStringTimer_Tick(object sender, EventArgs e)
        {
            //This function polls the serial for reading strings (Serial.println() is used on the Arduino)
            int bytesToRead = 0; //This variable stores the bytes to read. we can have max 2^32 bytes - 4 gigabytes

            serialStringTimer.Stop(); //while we process the data, we are not supposed to run the clock while doing the buffering

            try
            {
                bytesToRead = MCU_serialport.BytesToRead; //tries to check the amount of bytes in the serial buffer
            }
            catch
            {
                //not implemented yet(?)
            }

            if (MCU_serialport.IsOpen) //if the serial port is open (selected in the drop-down list)
            {
                if (bytesToRead != 0) //if there is something waiting on the serial port buffer
                {                    
                    incomingString = MCU_serialport.ReadExisting(); //read the existing data from the serial port

                    //receiveTextBox.AppendText("Incoming string: " + incomingString); //print it on the serial port

                    //incoming data is just being expanded with every tick reading, it will be copied and emptied somewhere else
                    incomingStringBuffer.Add(incomingString); //
                    
                }
                serialStringTimer.Start(); //Ticking is restarted after the above task is done
            }

            dt2 = DateTime.Now; //check the time now - this (dt2) is compared to dt1 in the next steps
            elapsedSeconds = ((TimeSpan)(dt2 - dt1)).TotalSeconds; //calculate the difference between the start time and the currently checked time

            if (elapsedSeconds > 3) //for now, the buffer holds the data for 3 seconds
            {
                //print to file
                printToFile(); // print (append) to file                

                //reset timer - 3 second counting starts over
                dt1 = DateTime.Now;
            }
        }

        private void printToFile()
        {
            //When the checkbox is checked, this function dumps the content of the fileBuffer into a txt file
            //If the streamwriter does not see the file, it creates it. If it exists, the next buffer dump will write to the end ("append")

            if (fileSaveCheckBox.Checked == true) //if file saving is enabled
            {
                if (binaryInputCheckBox.Checked) //24-bit binary data is expected - Serial.write(data, lenght)
                {
                    using (System.IO.StreamWriter sw = File.AppendText(filepath + "outputFile.txt"))
                    {
                        foreach (int number in fileBuffer) //dump all the lines into the file line by line
                        {
                            sw.WriteLine(number);
                        }
                    }
                    //clear buffer
                    fileBuffer.Clear(); //free up the buffer 
                                        //clearing the buffer should only happen if we saved the data into an external file
                                        //however if we let the fileBuffer grow, it might lead to problems.
                }
                else //string is expected - Serial.println(data)
                {
                    using (System.IO.StreamWriter sw = File.AppendText(filepath + "outputFile.txt"))
                    {
                        foreach (string text in incomingStringBuffer) //dump all the lines into the file line by line
                        {
                            sw.WriteLine(text);
                        }
                    }
                    //clear buffer
                    incomingStringBuffer.Clear();
                }
            }
        }
    }
}
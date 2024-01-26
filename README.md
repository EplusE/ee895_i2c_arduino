[![E+E_Logo](./images/epluse-logo.png)](https://www.epluse.com/en/)

# EE895 I2C with Arduino


![EE895](./images/EE895-co2-element.png)  


<!--[![button1](./images/learn-more.png)](https://www.epluse.com/en/products/co2-measurement/co2-sensor/ee895/)  -->
[![button2](./images/data-sheet.png)](https://downloads.epluse.com/fileadmin/data/product/ee895/datasheet_EE895.pdf) 



## QUICK START GUIDE  

### Components 
- EE895
- Arduino
- Breadboard 
- Wire jumper cable <br>

| Step |                                                                                                                                                             |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1    | Connect the EE895 sensor module with Arduino according to the following scheme: <br> [<img src="images/EE895_arduino.PNG" width="60%"/>](images/EE895_arduino.PNG)|
| 2    | Download and install the Arduino IDE (https://www.arduino.cc/en/software). Version >1.8.7 recommended.                                                            |
| 3    | Download the ZIP File of this project.|
| 4    | Open the arduino software.|
| 5    | Go to: <br>[<img src="images/add_library.png" width="550"/>](images/add_library.png) |
| 6    | Search for the downloaded ZIP File and open it.|
| 7    | Go to:<br>[<img src="images/open_file.png" width="500"/>](images/open_file.png)|
| 8    | Connect the Arduino to your PC via the USB cable. Select Board “Arduino Uno” and the appropriate COM-Port from Tools menu and upload the sketch |
| 9    | When the upload is finished, open the the "Serial Monitor" with the key combination (Control + Shift + M) or via the menu bar: <br> [<img src="images/serial_Monitor.png" width="400"/>](images/serial_Monitor.png) |


### Please note:
The Arduino Uno board has no hardware pull-up resistors for the I2C bus. These are enabled internally in the <br>
microcontroller library. In case of communication problems, pull-up resistors can be added externally.<br>
for example:<br>
[<img src="images/pull_ups.PNG" width="700"/>](images/pull_ups.PNG)



<br>

## License 
See [LICENSE](LICENSE).

'''
Copyright (c) 2018 Alireza Bahremand

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import serial
import math
# import time

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)


channel0 = "LUM CH0:"
channel1 = "LUM CH1:"
wind = "X:"
soilSensor1 = "SOIL SENSOR #1:"
soilSensor2 = "SOIL SENSOR #2:"
tempReading = "TEMP:"
humidityReading = "HUMIDITY:"
intruder = "INTRUDER:"

channel_0_value = 0
channel_1_value = 0
temp_value = 0
wind_direction = 0
humidity_value = 0
soil_sensor_1_value = 0
soil_sensor_2_value = 0
luxRatio = 0
lux = 0
intruderAlert = False


# ROW 1
html_intruder_alert_valid = """
    <!-- Laser -->
    <!-- Intruder -->
    <div class="col s6 m6">
    <div class="card">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="http://assets.nydailynews.com/polopoly_fs/1.160572.1314012566!/img/httpImage/image.jpg_gen/derivatives/article_750/alg-burglar-jpg.jpg">
    <span class="card-title" style="color: #2775ba;letter-spacing: 0.1em; text-transform: uppercase; font-size: 55px; font-weight: bold;">Oh shit</span>
    </div>
    <div class="card-content">
    <p>There is an intruder</p>
    </div>
    </div>
    </div>
    """

html_intruder_alert_notvalid = """
    <!-- Laser -->
    <!-- No intruder -->
    <div class="col s6 m6">
    <div class="card z-depth-4">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="yard_img/rainbow.jpg">
    <span class="card-title" style="color: #2775ba;letter-spacing: 0.1em; text-transform: uppercase; font-size: 55px; font-weight: bold;">Good Vibes</span>
    </div>
    <div class="card-content">
    <p>There is no intruder, the house is safe.</p>
    </div>
    </div>
    </div>
    """

html_lux_sunny = """
    <!-- Lux -->
    <!-- sunny -->
    <div class="col s6 m6">
    <div class="card z-depth-4">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="http://www.findgoodmovers.net/wp-content/uploads/2011/12/movers-Mesa-AZ.jpg">
    <span class="card-title" style="color: #2775ba;letter-spacing: 0.1em; text-transform: uppercase; font-size: 55px; font-weight: bold;">Sunny</span>
    </div>
    <div class="card-content">
    <p>It is sunny outside.</p>
    <p>The Lux Calculation is:    </p>
    """

html_lux_cloudy = """
    <!-- Lux -->
    <!-- Cloudy -->
    <div class="col s6 m6">
    <div class="card z-depth-4">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="yard_img/sunnyDay.jpg">
    <span class="card-title" style="color: #2775ba;letter-spacing: 0.1em; text-transform: uppercase; font-size: 55px; font-weight: bold;">Cloudy</span>
    </div>
    <div class="card-content">
    <p>It is cloudy outside.</p>
    <p>The Lux Calculation is:    </p>
    """
html_end_row_text = """    
    </div>
    </div>
    </div>
    </div>
    <!-- End of row -->
    """

# ROW 2
html_temp = """
    <!-- Temperature -->
    <div class="col s6 m4">
    <div class="card z-depth-2">
    <div class="card-image">
    <img style="width: 310px; height:207;" src="yard_img/temperature.jpg">
    <span class="card-title" style="color: #fff614;letter-spacing: 0.1em; text-transform: uppercase; font-size: 32px; font-weight: bold;">Temperature</span>
    </div>
    <div class="card-content">
    <p>The Temperature is:</p>
    """

html_close_container = """
    </div>
    </div>
    </div>
    """

html_pot1_soil_moist = """
    <!-- Moist Soil -->
    <div class="col s6 m4">
    <div class="card z-depth-2">
    <div class="card-image">
    <img style="width: 310px; height:207;" src="yard_img/moistSoil.jpg">
    <span class="card-title" style="color: #f30b1f;letter-spacing: 0.1em; text-transform: uppercase; font-size: 38px; font-weight: bold;">Pepper:</span>
    </div>
    <div class="card-content">
    <p>The soil in the pot of the plants is moist.</p>
    <p>The Moisture is:    </p>
    """

html_pot1_soil_dry = """
    <!-- Dry Soil -->
    <div class="col s6 m4">
    <div class="card z-depth-2">
    <div class="card-image">
    <img style="width: 310px; height:207;" src="yard_img/drySoil.jpg">
    <span class="card-title" style="color: #f30b1f;letter-spacing: 0.1em; text-transform: uppercase; font-size: 38px; font-weight: bold;">Pepper:</span>
    </div>
    <div class="card-content">
    <p>The soil in the pot of the plant is dry.</p>
    <p>The Moisture is:    </p>
    """


html_pot2_soil_moist = """
    <!-- Moist Soil -->
    <div class="col s6 m4">
    <div class="card z-depth-2">
    <div class="card-image">
    <img style="width: 310px; height:207;" src="yard_img/moistSoil.jpg">
    <span class="card-title" style="color: #f37c1c;letter-spacing: 0.1em; text-transform: uppercase; font-size: 32px; font-weight: bold;">Parsley:</span>
    </div>
    <div class="card-content">
    <p>The soil in the pot of the plants is moist.</p>
    <p>The Moisture is:    </p>
    """

html_pot2_soil_dry = """
    <!-- Dry Soil -->
    <div class="col s6 m4">
    <div class="card z-depth-2">
    <div class="card-image">
    <img style="width: 310px; height:207;" src="yard_img/drySoil.jpg">
    <span class="card-title" style="color: #f37c1c;letter-spacing: 0.1em; text-transform: uppercase; font-size: 32px; font-weight: bold;">Parsley:</span>
    </div>
    <div class="card-content">
    <p>The soil in the pot of the plant is dry.</p>
    <p>The Moisture is:    </p>
    """

# ROW 3
html_windy = """
    <!-- Wind -->
    <div class="col s6 m6">
    <div class="card z-depth-3">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="https://www.madeiraislandnews.com/wp-content/uploads/2017/11/windy_golf.png">
    <span class="card-title" style="color: #2775ba;letter-spacing: 0.1em; text-transform: uppercase; font-size: 32px; font-weight: bold;">Breeze</span>
    </div>
    <div class="card-content">
    <p>It is windy!</p>
    </div>
    </div>
    </div>
    """

html_not_windy = """
    <!-- Not windy -->
    <div class="col s6 m6">
    <div class="card z-depth-3">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="http://s629411786.onlinehome.us/wp-content/uploads/2011/11/img_2565.jpg">
    <span class="card-title" style="color: #2775ba;letter-spacing: 0.1em; text-transform: uppercase; font-size: 32px; font-weight: bold;">No Wind</span>
    </div>
    <div class="card-content">
    <p>It is not windy outside!</p>
    </div>
    </div>
    </div>
    """

html_humidity = """
    <!-- Humidity -->
    <div class="col s6 m6">
    <div class="card z-depth-3">
    <div class="card-image">
    <img style="width: 476px; height: 365px;" src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/83/Cloud_forest_mount_kinabalu.jpg/1200px-Cloud_forest_mount_kinabalu.jpg">
    <span class="card-title">Humidity:</span>
    </div>
    <div class="card-content">
    <p>It is humid outside:</p>
    <p>The Humidity is:    </p>
    """

html_end_of_doc = """
    </body>
    <!-- end of document -->
    </html>
    """


html_str = """
    <!DOCTYPE html>
    <html>
    
    <head>
    <!-- MATERIALIZE -->
    <!-- Compiled and minified CSS -->
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/materialize/0.100.2/css/materialize.min.css">
    <!-- Compiled and minified JavaScript -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/materialize/0.100.2/js/materialize.min.js"></script>
    <title>The Bahremand Household Backyard</title>
    </head>
    
    <body>
    <nav>
    <div class="nav-wrapper" style="background-color: #79d650;">
    <a href="#" class="brand-logo center">456 Project</a>
    <ul id="nav-mobile" class="left hide-on-med-and-down">
    </ul>
    </div>
    </nav>
    """

html_row_1 = """
    <!-- 1st Row -->
    <div class="container row" style="margin-top: 50px;">
    """

html_row_2 = """
    <!-- 2nd Row -->
    <div class="container row">
    """

html_row_3 = """
    <!-- 3rd Row -->
    <div class="container row" style="margin-bottom: 25px;">
    """


def calculateLux(luxRatio):
    if (luxRatio > 0 and luxRatio <= 0.52):
        luxVal = 0.0315 * channel_0_value - 0.0593 * channel_0_value * (math.pow(luxRatio, 1.4))
    elif (luxRatio > 0.52 and luxRatio <= 0.65):
        luxVal = 0.0229 * channel_0_value - 0.0291 * channel_1_value
    elif (luxRatio > 0.65 and luxRatio <= 0.80):
        luxVal = 0.0157 * channel_0_value - 0.0180 * channel_1_value
    elif (luxRatio > 0.80 and luxRatio <= 1.30):
        luxVal = 0.00338 * channel_0_value - 0.00260 * channel_1_value
    elif (luxRatio > 1.30):
        luxVal = 0
    return luxVal


def parseReadings(serialText, strip):
    # Get rid of whitespace.
    serialText = serialText.replace(strip, "")
    serialText = serialText.replace(" ", "")
    # Assign our sensor read value
    # sensorValue = float(''.join(serialText.split()))
    sensorValue = float(serialText)
    return sensorValue


# NOTE: This doesn't work because of context manager, it is automatically
# with open("OutputSPI.txt", "w") as text_file:
# Declare all global values to read parsed input.
# Input parsed by looking for substrings within each
# line of serial input.
def grabSensorValues():
    global html_str
    global channel_0_value
    global channel_1_value
    global temp_value
    global wind_direction
    global humidity_value
    global soil_sensor_1_value
    global soil_sensor_2_value
    global luxRatio
    global lux
    global intruderAlert

    # /var/www/html/
    while True:
        # Read characters until EOL.
        read_serial = ser.readline()
        print(read_serial)
        # ROW1
        if (intruder in read_serial):
            if ("true" in read_serial):
                intruderAlert = True
#           print("Found intruder text")
        elif (channel0 in read_serial):
            channel_0_value = parseReadings(read_serial, channel0)
        elif (channel1 in read_serial):
            channel_1_value = parseReadings(read_serial, channel1)
            if (channel_0_value != 0):  # if input is not 0 calculate lux
                luxRatio = (channel_1_value / channel_0_value)
            lux = calculateLux(luxRatio)
#            print("Lux calculated: " + str(lux))
        # ROW 2
        elif (tempReading in read_serial):
            temp_value = parseReadings(read_serial, tempReading)
        elif (soilSensor1 in read_serial):
            soil_sensor_1_value = parseReadings(read_serial, soilSensor1)
        
        elif (soilSensor2 in read_serial):
            soil_sensor_2_value = parseReadings(read_serial, soilSensor2)
        # ROW 3
        elif (wind in read_serial):
            wind_direction = parseReadings(read_serial, wind)
        elif (humidityReading in read_serial):
            humidity_value = parseReadings(read_serial, humidityReading)
            break

# Open/write html file, w parameter to overwrite.
# Method properly stitches HTML file together by
# splicing HTML strings with appropriate parsed input
# from global variables.
if __name__ == '__main__':
    grabSensorValues()
    with open("/var/www/html/sensorReadings.html", "w") as file:
        html_str += html_row_1
        if (intruderAlert):
            html_str += html_intruder_alert_valid
        else:
            html_str += html_intruder_alert_notvalid
        if (lux > 3):
            html_str += html_lux_sunny
        else:
            html_str += html_lux_cloudy
        # Print lux and end of row
        html_str += "<h5>" + str(lux) + "</h5>"
        html_str += html_end_row_text
        # Row 2
        html_str += html_row_2
        html_str += html_temp
        html_str += "<h5>" + str(temp_value) + "</h5>"
        html_str += html_close_container
        # Check moisture
        if (soil_sensor_1_value > 140):
            html_str += html_pot1_soil_moist
        else:
            html_str += html_pot1_soil_dry
        html_str += "<h5>" + str(soil_sensor_1_value) + "</h5>"
        html_str += html_close_container
        # 2nd Plant
        if (soil_sensor_2_value > 140):
            html_str += html_pot2_soil_moist
        else:
            html_str += html_pot2_soil_dry
        # Print soil moisture and end of row.
        html_str += "<h5>" + str(soil_sensor_2_value) + "</h5>"
        html_str += html_end_row_text
        # Row 3
        html_str += html_row_3
        # Wind
        if (wind_direction < 80):
            html_str += html_windy
        else:
            html_str += html_not_windy
        # Humidity
        html_str += html_humidity
        html_str += "<h5>" + str(humidity_value) + "</h5>"
        html_str += html_end_row_text
        # Append end of html file
        html_str += html_end_of_doc
        # Write file
        file.write(html_str)

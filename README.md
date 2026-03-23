# Cyber-Clock
Main funcionality is displaying time, weather and adequete to weather avatar on e-Paper display \
Planned to add: 
1. Data base for data from BME280 
2. Files for 3d printing the case

# Hardware
Program was built on esp32s3 and waveshare 5.79 inch e-Paper display. But it should work on any esp32

# To run
You need to have espressif ide installed and on linux run: \
`
idf.py set-target <ESP> 
`
\
`
idf.py -p <PORT> flash monitor 
` \
You also need to add api key for open weather map api and wifi parameters in main.c, if you want weather functionality. 

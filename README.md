# time_weather_skill

# Inputs:
Type_of_information/city/day/information_required/display

## Type of information:
+ weather: Displays the weather and return info
+ time: Return the actual state time (day or night)
+ comment: Return a joke

## City:
If not specified, write: 'default'
By default: Madrid (in future use gps)


## Day:
+ 'week': shows all week info
+ 'today' or '0': shows today info
+ 'tomorrow' or '1'
+ '2'
+ '3'
+ ...
+ '9'


## Information_required:
+ 'basic': Basic info
++ Current (today):
+++ last updated (current/last_updated)
+++ city name (location/name)
+++ country name (location/country)
+++ date (forecast/forecastday[0]/date)
+++ temp_c (current/temp_c)
+++ text (current/condition/text) Example:'Partially cloud'
+++ code (current/condition/code)
+++ is_day (current/is_day or timeClass)
++ Forecast:
+++ last updated (current/last_updated)
+++ city name (location/name)
+++ country name (location/country)
+++ date (forecast/forecastday[i]/date)
+++ avgtemp_c (forecast/forecastday[i]/day/avgtemp_c)
+++ text (forecast/forecastday[i]/day/condition/text) Example:'Partially cloud'
+++ code (forecast/forecast[i]/day/condition/code)

+ 'advanced': Advanced info
++ Current (today):
+++ Previous list
+++ precip_mm (current/precip_mm)
+++ ...
++ Forecast:
+++ Previous list
+++ mintemp_c (forecast/forecastday[i]/day/mintemp_c)
+++ maxtemp_c (forecast/forecastday[i]/day/maxtemp_c)
+++ totalprecip_mm (forecast/forecastday[i]/day/totalprecip_mm)
+++ ...

+ specific_info: specify the info needed


## Display:
Each number corresponds to one variable:
+ '101' (Screen, movement, voice)
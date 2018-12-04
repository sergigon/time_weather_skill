# time_weather_skill

# Inputs:
Type_of_information/city/display/day/information

## Type of information:
+ weather: Displays the weather and return info
+ time: Return the actual state time (day or night)
+ comment: Return a joke

## City:
If not specified, write: 'default'
By default: Madrid (in future use gps)

## Display:
Each number corresponds to one variable:
+ '101' (Screen, movement, voice)

## Day:
+ 'week': shows all week info
+ 'today' or '0': shows today info
+ 'tomorrow' or '1'
+ '2'
+ '3'
+ ...
+ '9'

## Information:
+ 'standard': Basic info
+ 'advanced': Advanced info
+ specific_info: specify the info needed ¿?

### Information variables:
++ Info (Current)
+++ last updated (current/last_updated)
+++ city name (location/name)
+++ country name (location/country)
+++ date (forecast/forecstday[0]/date)
+++ temp_c (current/temp_c)
+++ text (current/condition/text) Example:'Partially cloud'
+++ is_day (current/is_day or timeClass)
########## Advanced ###########
+++ precip_mm (current/precip_mm)
+++ ...
###############################

++ Important info (Forecast)
+++ last updated (current/last_updated)
+++ city name (location/name)
+++ country name (location/country)
+++ date (forecast/forecstday[i]/date)
+++ avgtemp_c (forecast/forecast[i]/day/avgtemp_c)
+++ mintemp_c (forecast/forecast[i]/day/mintemp_c)
+++ maxtemp_c (forecast/forecast[i]/day/maxtemp_c)
+++ text (forecast/forecast[i]/day/condition/text) Example:'Partially cloud'
+++ code (forecast/forecast[i]/day/condition/code)
########## Advanced ##########
+++ totalprecip_mm (forecast/forecastday[i]/day/totalprecip_mm)
+++ ...
##############################
# time_weather_skill

# Description
Skill for time and weather.

It can provide info about the state of the day (day or night), or info about the weather forecast.

## Time info
It returns the state info: 'day' or 'night'.

Source:
+ [Astral](https://pypi.org/project/astral/): Uses **local** data

## Weather info
It returns weather info.

Sources:
+ [Apixu](https://www.apixu.com/): Uses **URL** data

# Goal
## Goal structure
`'Type_of_information/city/day/information_required/display'`

Examples:
+ `'weather/madrid/2/basic/101'`
+ `'time/paris'`

## Type of information:
+ **weather**: Displays the weather and return info
+ **time**: Return the actual state time (day or night)
+ **comment**: Return a joke

## City:
City where to check the time/weather.
Some cities may not be found.

For **weather**, it can be specified the country.

Example: **`'Madrid, Spain'`**

### Lists of available cities
#### Time:
+ [Astral](https://astral.readthedocs.io/en/stable/index.html#cities)
#### Weather:
+ Apixu: Most of the world cities

## Day:
Posible inputs:
+ `'today'` or `'0'`: shows today info
+ `'tomorrow'` or `'1'`: shows tomorrow info
+ `'2'`: shows info within 2 days
+ `'3'`
+ ...
+ `'6'`


## Information_required:
Introduce the names of the info you need. To get more than one variable, specify it with separating by commas.

Example: **`'last_update,temp_c'`**

You can get directly a list, writting `'basic'` or `'advanced'` instead.

The list of possible outputs are the following:

+ `'basic'`: Basic info
  + Current:
    + `'last_updated'`: Last update forecast date
    + `'city_name'`: Name of the city
    + `'country_name'`: Name of the country
    + `'date'`: Current date
    + `'temp_c'`: Temperature in celsius degrees
    + `'text'`: Weather condition (Example:'Partially cloud')
    + `'code'`: Code number for icons, it depends on the ´text´
    + `'is_day'`: Day or night
  + Forecast:
    + `'last updated'`: Last update forecast date
    + `'city name'`: Name of the city
    + `'country name'`: Name of the country
    + `'date'`: Forecast day
    + `'avgtemp_c'`: Average temperature in celsius degrees
    + `'text'`: Weather condition text (Example:'Partially cloud')
    + `'code'`: Weather condition code, used for the icons

+ `'advanced'`: Advanced info
  + Current:
    + -Previous list-
    + `'precip_mm'`: Precipitation amount in millimeters
  + Forecast:
    + -Previous list-
    + `'mintemp_c'`: Minimum temperature in celsius
    + `'maxtemp_c'`: Maximum temperature in celsius for the day
    + `'totalprecip_mm'`: Total precipitation in milimeters

This output can be extendable modifing each weather class inside the skill code.


## Display:
Each number corresponds to one variable:
+ '101' (Screen, movement, voice)

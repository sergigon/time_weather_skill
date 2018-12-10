# time_weather_skill

## Overview

Skill for time and weather.

It can provide info about the state of the day (day or night), or info about the weather forecast.

### Time info
It returns the state info: 'day' or 'night'.

Source:
+ [Astral](https://pypi.org/project/astral/): Uses **local** data

### Weather info
It returns weather info.

Sources:
+ [Apixu](https://www.apixu.com/): Uses **URL** data

## Installation
### Astral:
`pip install astral`

## Run

Explain in detail how to run the system: nodes, launch files, etc.

## Goal
### Goal structure
`'Type_of_information/city/day/information_required/display'`

Examples:
+ `'weather/madrid/2/basic/101'`
+ `'time/paris'`

### Type of information
Type of info needed.
Posible inputs:
+ **weather**: Displays the weather and return info
+ **time**: Return the actual state time (day or night)
+ **comment**: Return a joke

### City
City where to check the time/weather.

For **weather**, it can be specified the country.
Example: **`'Madrid, Spain'`**

Some cities may not be found.
#### Lists of available cities:
+ **Time**
  + [Astral](https://astral.readthedocs.io/en/stable/index.html#cities)
+ **Weather**
  + Apixu: Most of the world cities

### Day
Forecast day.

Posible inputs:
+ `'today'` or `'0'`: shows today info
+ `'tomorrow'` or `'1'`: shows tomorrow info
+ `'2'`: shows info within 2 days
+ `'3'`
+ ...
+ `'6'`


### Information_required
Names of the info you need. To get more than one variable, specify it with separating by commas.

Example: **`'last_update,temp_c'`**

You can get directly a list, writting `'basic'` or `'advanced'` instead.

The list of possible inputs are the following:
+ `'last_updated'`: Last update forecast date
+ `'city_name'`: Name of the city
+ `'country_name'`: Name of the country
+ `'date'`: Current date
+ `'text'`: Weather condition (Example:'Partially cloud')
+ `'code'`: Code number for icons, it depends on the ´text´
Only for current:
+ `'temp_c'`: Temperature in celsius degrees
+ `'is_day'`: Day or night
+ `'precip_mm'`: Precipitation amount in millimeters
Only for forecast:
+ `'avgtemp_c'`: Average temperature in celsius degrees
+ `'mintemp_c'`: Minimum temperature in celsius
+ `'maxtemp_c'`: Maximum temperature in celsius for the day
+ `'totalprecip_mm'`: Total precipitation in milimeters

+ `'basic'`: Basic info. It includes the following data:
  + Current: `'last_updated'`, `'city_name'`, `'country_name'`, `'date'`, `'temp_c'`, `'text'`, `'code'`, `'is_day'`
  + Forecast: `'last_updated'`, `'city_name'`, `'country_name'`, `'date'`, `'avgtemp_c'`, `'text'`, `'code'`
+ `'advanced'`: Advanced info. It includes the following data:
  + Current: `Basic list`, `'precip_mm'`
  + Forecast: `Basic list`, `'mintemp_c'`, `'maxtemp_c'`, `'totalprecip_mm'`

This input can be extendable modifing each weather class inside the skill code.

### Display
Each number corresponds to one variable:
+ '101' (Screen, movement, voice)

## ROS API

### Node name

Brief description of the node

#### Published Topics(*message_package/MessageName*)

Description

#### Subscribed Topics(*message_package/MessageName*)

Description

#### Services(*service_package/ServiceName*)

Description

#### Actions(*action_package/ActionName*)

Description

#### Parameters	(*param_type*, default: *value*)

Description

## LICENSE

La UC3M es titular en exclusiva de los derechos de propiedad intelectual de dicho software, y acepta que el software se proporcione para uso exclusivo dentro de las tareas académicas, y su usuario no está por tanto autorizado a ningún otro uso indebido ajeno, entre estos, a título enunciativo pero no limitativo, a realizar ninguna reproducción, fijación, distribución, comunicación pública, ingeniería inversa, ni transformación sobre dicho software, siendo el propio usuario el responsable de cualquier uso indebido y de las consecuencias que pudieran derivarse de sus actos.

The UC3M is the exclusive owner of the intellectual property rights of this software, and accepts that the software is provided for exclusive use within the academic tasks, and its user is therefore not authorized to any other unauthorized use, among these, for example, but not limited to, make any reproduction, fixation, distribution, public communication, reverse engineering, or transformation on said software, being the user himself responsible for any misuse and the consequences that may arise from their actions.

## ACKNOWLEDGEMENTS

![RoboticsLab](http://ieee.uc3m.es/images/thumb/b/b6/Roboticslab_text_new.jpg/128px-Roboticslab_text_new.jpg)
![UC3M](http://ieee.uc3m.es/images/thumb/6/6b/Logo_uc3m_letras.png/256px-Logo_uc3m_letras.png)

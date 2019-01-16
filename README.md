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
`'city/forecast_type/day/information_required/display'`

Examples:
+ `'madrid/forecast_type/tomorrow/basic/101'`
+ `'sydney/current/0/is_day/011'`

### Type of information
Type of info needed.
Posible inputs:
+ **weather**: Displays the weather and return info
+ **time**: Return the actual state time (day or night)
+ **comment**: Return a joke

### Forecast Type
+ **current**: Returns the current weather.
Apixu act.: 10:30 (10:45), 10:45 (11:05), 13:45 (13:56), 17:45 (18:00), 19:00 (19:17), 19:15 (19:19), 19:45 (19:52), 20:00 (20:23), 20:15 (20:27), 20:30 (20:47), 20:45 (20:47), 21:00 (5:50)
+ **forecast**: Returns the forecast.

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
Names of the info you need. To get more than one variable, specify it separating by commas.

Example: **`'last_update,temp_c'`**

You can get directly a list, writting `'basic'` or `'advanced'` instead.

The list of possible inputs are the following:

Current parameters:
+ `'date'`: Current date ('2018-11-08')
+ `'temp_c'`: Temperature in celsius degrees (51.8)
+ `'is_day'`: Day or night (1 or 0)
+ `'precip_mm'`: Precipitation amount in millimeters (0.2)
+ `'text'`: Weather condition ('Partially cloud')
+ `'code'`: Code number for icons, it depends on the ´text´ (1003)
+ `'last_updated'`: Last update forecast date ('2018-11-08 17:45')

Forecast parameters:
+ `'date'`: Forecast date ('2018-11-08')
+ `'avgtemp_c'`: Average temperature in celsius degrees (10.7)
+ `'mintemp_c'`: Minimum temperature in celsius (8.3)
+ `'maxtemp_c'`: Maximum temperature in celsius for the day (12.9)
+ `'totalprecip_mm'`: Total precipitation in milimeters (0.04)
+ `'text'`: Weather condition ('Light rain shower')
+ `'code'`: Code number for icons, it depends on the ´text´ (1240)
+ `'last_updated'`: Last update forecast date ('2018-11-08 17:45')

Common parameters:
+ `'city_name'`: Name of the city ('Madrid')
+ `'country_name'`: Name of the country ('United_Kingdom')

Lists:
+ `'basic'`: Basic info. It includes the following data:
  + Current: `'date'`, `'temp_c'`, `'is_day'`, `'text'`, `'code'`, `'city_name'`, `'country_name'`, `'last_updated'`
  + Forecast: `'date'`, `'avgtemp_c'`, `'text'`, `'code'`, `'city_name'`, `'country_name'`, `'last_updated'`
+ `'advanced'`: Advanced info. It includes the following data:
  + Current: `Basic list`, `'precip_mm'`
  + Forecast: `Basic list`, `'mintemp_c'`, `'maxtemp_c'`, `'totalprecip_mm'`

This input can be extendable modifing each weather class inside the code.

### Display
Each number corresponds to one variable:
+ '101' (Screen, movement, voice)

## Introducing new weather sources
### Local sources
### Online sources
#### Parameters csv table
To make url requests to new sources, there must be some parameters that have to be specified. Each url source need different parameters.

These parameters must be modified in the `weather_sources_params.csv`.

At the end of the table there are **two examples**.

##### source
Name given to the new source. It must be at the **beginning** of the new rows. created.
##### forecast_type
Used to specify the 'current' or 'forecast' parameters.

Some sources make a distinction when making the requests between current weather or forecast. To that end this column allows to specify different parameters depending on it.

If the source only needs one only request for both 'current' and 'forecast', then leave it blank.

##### url
Url path for making the request.

##### params
List of parameters needed to make the request.

In the first column specify the keys, and in the next column write the corresponding values.

**IMPORTANT**: Use the following values if you want them to be filled automatically (recommended):
+ `__location`: location specified in the code
+ `__lang`: languaje specified in the code

##### extra_info
List of extra info that can be useful for the code or for documentation purposes.

+ limit_forecast_days: Limit of forecast days of the source.
+ limit_calls: Limit calls.

In the first column specify the keys, and in the next column write the corresponding values.

#### Error requests
Go to weatherClass.py:
	+ Go to Weather class and go to the _URL_request() method. Add the exceptions wanted.

## Introducing new info parameters
Go to weather_format_changer.py file:
+ Add the new info parameters in the standard dictionary list.
+ Add the corresponding info parameters in the source dictionaries lists. If needed any variable list (for forecast mainly) initialize it at the beggining of the code.

## Change info required lists
Go to weatherClass.py file:
+ To change the content of the info required lists change the content of the _INFO_(LISTNAME)_LIST at the beginning of the class.
+ To add a new list create the new list with the name _INFO_(LISTNAME)_LIST at the beginning of the class and add the info parameters wanted. Remember to distinguish between 'forecast' and 'current'. Then in the _get_info method:

```
if('(listname)' in info_required_list): # Check if (listname) list is requested
	info_required_list.remove('(listname)')
	info_required_list.extend(self._INFO_(LISTNAME)_LIST[forecast_type][:]) # Replace the list with the (listname) list
```


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

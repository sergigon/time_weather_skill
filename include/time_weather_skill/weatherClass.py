#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"


import rospy
import requests # URL requests
import datetime # Gets actual date

from time_weather_skill.create_json import CreateJson
from time_weather_skill.weather_format_changer import source2standard
from time_weather_skill.datetime_manager import DatetimeManager
from time_weather_skill.timeClass import TimeAstral

pkg_name = 'time_weather_skill'

class Weather():

    """
    Weather class.
    """

    # Variables
    _GOAL_MAX_SIZE = 5 # Max size of the weather goal
    _WEATHER_FILENAME = 'weather' # Name of the file to store the weather data

    # Lists
    _SOURCE_LIST = ['apixu', 'source1', 'source2'] # List of the sources
    _UPDATE_HOURS = [23, 19, 14, 9, 4] # List of hours for current request
    _INFO_BASIC_LIST_CURRENT = ['date', 'temp_c', 'is_day', 'text', 'code', 'city_name', 'country_name', 'last_updated'] # Basic current list
    _INFO_BASIC_LIST_FORECAST = ['date', 'avgtemp_c', 'text', 'code', 'city_name', 'country_name', 'last_updated'] # Basic advanced list
    _INFO_ADVANCED_LIST_CURRENT = _INFO_BASIC_LIST_CURRENT[:]
    _INFO_ADVANCED_LIST_FORECAST = _INFO_BASIC_LIST_FORECAST[:]
    _INFO_ADVANCED_LIST_CURRENT.extend(['precip_mm']) # Advanced current list
    _INFO_ADVANCED_LIST_FORECAST.extend(['mintemp_c', 'maxtemp_c', 'totalprecip_mm']) # Advanced forecast list
    _TIMECLASS_LIST = ['is_day'] # List of the TimeClass parameters

    def __init__(self):
        """
        Init method.
        """

        # Weather sources
        
        self._create_json = CreateJson() # CreateJson object

    def _save_json(self, input_dic, extra_dic={}):
        """
        Save weather dictionary in a Json file. It can be added
        extra information.

        IMPORTANT: Json data must be in STANDARD format.

        @param input_dic: Weather dictionary in STANDARD format.
        @param extra_dic: Dictionary with extra info to save.
        """

        # Add extra info
        input_dic.update(extra_dic)

        # Define filename
        filename = self._WEATHER_FILENAME + '_' + input_dic['common']['city_name'].lower() + '_' +  input_dic['common']['country_name'].lower()
 
        # Write the data in the JSON file
        self._create_json.write(input_dic, filename) # Write weather info into JSON file

    def _location(self, location):
        """
        Divide the location in city_name and country_name.

        @param location: City to get weather. Format: 'Madrid' or 'Madrid, Spain'.

        @return city_name: Name of the city.
        @return country_name: Name of the country.
        """

        # Initialize variables
        city_name, country_name = '', ''

        # Checks if location has specified country
        n_commas = location.find(',')
        if(n_commas <= 0): # NOT specified country
            city_name = location
        else: # Specified country
            city_name, country_name = location.split(",")
            if(country_name[0] == ' '): # Removes space character if necessary
                country_name = country_name[1:]

        return city_name, country_name

    def _fix_date(self, date):
        """
        Fix the date and converts to int.

        @param date: date to fix.
        """

        if (date == 'today'):
            date = '0'
        elif (date == 'tomorrow'):
            date = '1'

        date = int(date) # Converts to int

        return date

    def _URL_request(self, url_forecast, params):
        """
        URL request method.

        @param params: Dictionary with the URL equest params.
        
        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Make URL request
        try:
            r = requests.get(url_forecast, params = params)
            # Get the data from the URL in JSON format
            if 'error' in r.json(): # ERROR in the request
                print('URL request ERROR')
                print(r.json()['error']['message']) # Print the error
                return -1, {}
            else: # NO error in the request
                print('URL request succeded')
                return 0, r.json()
        except:
            print('Connection ERROR')
            return -1, {}

    def _local_request(self, city_name, country_name = ''):
        """
        Request local method. Search by the city and country names.

        @param city_name: City to calculate weather.
        @param country_name: Country to calculate weather.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Initialize results
        result = -1
        result_info_dic = {}
        found = False

        # File to search (Format ex: 'weather_madrid_spain.json')
        file_name = self._WEATHER_FILENAME
        file_name = file_name + '_' + city_name.lower()
        if (country_name != ''): # Country specified
            file_name = file_name + '_' + country_name.lower()

        # Search json files in the data folder
        list_json = self._create_json.ls_json()

        # Search file in the list
        file_name_len = len(file_name)
        for list_i in list_json:
            if(file_name == list_i[:file_name_len]):
                file_name = list_i[:-5] # Get filename without the '.json' string
                # Check if JSON files exists
                result_info_dic = self._create_json.load(file_name) # Load JSON file
                found = True
                break

        if(found and result_info_dic != -1):
            print('\'' + file_name + '\'' + ' file found')
            result = 0
        else:
            result, result_info_dic = -1, {}
            rospy.logwarn("Local request ERROR: File not found")

        return result, result_info_dic
        
        '''
        make_req = False # Indicate to fill the data
        if (data_json != -1 and data_json_check != -1): # JSON files exists
            # Check all parameters exist
            #try:
            # Check if 'city' and 'lang' coincide
            if(location.lower() == data_json_check['city'].lower() and lang.lower() == data_json_check['lang'].lower()): # 'city' and 'lang' coincide
                now = datetime.datetime.now() # Get current date
                # Check if date is updated
                if(data_json_check['day'] == str(now.day) and data_json_check['month'] == str(now.month) and data_json_check['year'] == str(now.year)): # Already updated
                    print('Info in local found: "' + data_json_check['city'].lower() + '", "' + data_json_check['lang'].lower() + '"')
                    # Current is selected
                    if (forecast == 'current'):
                        print('Checking if current weather is updated...')
                        hour = -1 # Hour used to update
                        for update_hour in self.update_hours: # Search the hour in the list
                            hour = update_hour
                            if(update_hour < now.hour): # When find the hour it stops
                                break
                        # Check if current weather is updated
                        if(data_json_check['hour'] == str(hour)): # Already updated
                            print('Current weather already updated')
                            make_req = True
                        else:
                            print('Current weather NOT updated')
                    else:
                        print('hola')
                        make_req = True
                else:
                    rospy.logwarn("Local request ERROR: File not updated")
            else:
                rospy.logwarn("Local request ERROR: 'city' or 'lang' do not coincide")
            #except:
            #   rospy.logwarn("Local request ERROR: Parameters not existing")
        else:
            rospy.logwarn("Local request ERROR: JSON files do not exist")

        if(make_req):
            self.__data = data_json # Get JSON data
            print('Local request succeded')
            return True
        else:
            print('Local request not available')
            return False
        '''


    def _get_info(self, location, forecast_type, date, info_required):
        """
        Get the info specified. It uses local methods or make requests to that end.

        @param location: City to get weather. Format: 'Madrid' or 'Madrid, Spain'.
        @param forecast_type: 'forecast' or 'current'
        @param date: Date for the weather.
        @param info_required: Type of info_required needed.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Initialize result
        result, result_info_dic = -1, {}
        # Initialize variables
        standard_weather_dic = {}

        # Converts to int the date for the result_info dictionary
        date = self._fix_date(date)

        # Prepare info_required list
        info_required = info_required.replace(' ','') # Remove spaces from the string
        info_required_list = info_required.split(",") # Separate the goal in various parts

        if('advanced' in info_required_list): # Check if advanced list is requested
            if(forecast_type == 'current'):
                info_required_list = self._INFO_ADVANCED_LIST_CURRENT[:] # Replace the list with the advanced list
            if(forecast_type == 'forecast'):
                info_required_list = self._INFO_ADVANCED_LIST_FORECAST[:] # Replace the list with the advanced list

        if('basic' in info_required_list): # Check if basic list is requested
            if(forecast_type == 'current'):
                info_required_list = self._INFO_BASIC_LIST_CURRENT[:] # Replace the list with the basic list
            if(forecast_type == 'forecast'):
                info_required_list = self._INFO_BASIC_LIST_FORECAST[:] # Replace the list with the basic list


        ############### Fill the result_info dictionary ###############
        request = False # Indicates if a request has been done

        # Search in the info required list
        for info_required_i in info_required_list:
            found = -1
            # ############## Take data from TimeAstral ############## #
            if(forecast_type == 'current' and info_required_i in self._TIMECLASS_LIST): # This info can be taken with TimeClass
                city_name, _ = self._location(location) # # Get the city name form location variable
                print('Searching \'' + info_required_i + '\' in TimeAstral: ' + city_name)
                time_var = TimeAstral()
                found, data_time = time_var.get_info(city_name, info_required_i) # Get TimeAstral info
                if(found != -1):
                    result_info_dic.update({info_required_i: data_time})
                else:
                    rospy.logwarn("TimeAstral ERROR: Data not found")

            # ######## Take data from local or URL requests ######### #
            if(found == -1): # Parameter not found with other source
                # Make only ONE request
                if(not request): # A request has not been done
                    result, standard_weather_dic = self._request_weather(location, forecast_type, date, info_required) # Get the weather info
                    request = True
                    if(result == -1): # If there is a request error, it gets out
                        return result, {}
                
                # Checks if info required exists in forecast_type list
                if info_required_i in standard_weather_dic[forecast_type]:
                    result_info_dic.update({info_required_i: standard_weather_dic[forecast_type][info_required_i]})
                # Checks if info required exists in 'forecast'/'forecastday' list
                if forecast_type == 'forecast':
                    if info_required_i in standard_weather_dic[forecast_type]['forecastday'][date]:
                        result_info_dic.update({info_required_i: standard_weather_dic[forecast_type]['forecastday'][date][info_required_i]})
                # Checks if info required exists in 'common' list
                if 'common' in standard_weather_dic:
                    if info_required_i in standard_weather_dic['common']:
                        result_info_dic.update({info_required_i: standard_weather_dic['common'][info_required_i]})
            print(result_info_dic)
        result = 0

        return result, result_info_dic

    def _request_weather(self, location, forecast_type, date, info_required):
        """
        Get weather from local or URL sources, and save it if it success.

        @param location: City to get weather. Format: 'Madrid' or 'Madrid, Spain'.
        @param forecast_type: 'forecast' or 'current'
        @param date: Date for the weather.
        @param info_required: Type of info_required needed.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Initialize results
        result, result_info_dic = -1, {}
        # Initialize variables
        lang = 'es'
        updated = False # Variable to update local info

        ################ Checks if it is in local ################
        # ############# Get city and country names ############# #
        city_name, country_name = self._location(location)

        # ################# Make local request ################# #
        print("Making local request: " + city_name + ", " + country_name)
        result, result_info_dic = self._local_request(city_name, country_name)

        # ########### Check if local data is updated ########### #
        if(result == 0):
            # Manage last_update variable
            last_update = DatetimeManager(result_info_dic[forecast_type]['last_updated'])
            print('Last update (' + forecast_type + '): ' + last_update.date())
            # Current datatime
            now = datetime.datetime.now() # Get current date
            print('Local date: ' + str(now.year) + '-' + str(now.month) + '-' + str(now.day))
            if(now.year == last_update.year() and now.month == last_update.month() and now.day == last_update.day()): # Date updated
                print('Date is updated')
                if(forecast_type == 'forecast'):
                    pass

                if(forecast_type == 'current'):
                    pass
            else: # Date NOT updated
                rospy.logwarn("Local request ERROR: File not updated")

            return result, result_info_dic

            
        

        '''
        if True:
            read local
            check if it is updated
            if(forecast_type == 'forecast'):
                check if forecast is updated
            elif(forecast_type == 'current'):
                check if current is updated
        
            if(updated == True):
                return 0, result_info_dic
            else:
                print(forecast_type + ' not updated')
        # else:
            print('Info not found in local')

        '''


        ################## Make URL requests #####################
        for source in self._SOURCE_LIST: # Selects the source from the source list
            print("Making URL request to: '" + source + "'")

            ################# CHANGE CODE HERE ###################
            # >> Each source must get the FORECAST and        << #
            # >> CURRENT json info and SAVE it in the         << #
            # >> variable 'result_info_dic'                   << #
            # \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ #

            ########### Apixu source ###########
            if(source == 'apixu'):
                ################### NOTES ###################
                # It only make ONE request, since           #
                # forecast requests gives also current info #
                #############################################

                # Params
                url_forecast = 'http://api.apixu.com/v1/forecast.json'
                params = {
                    'key': '9838870ce6324daf95d161656180811',
                    'q': location,
                    'days': 7, # Forecast range
                    'lang': lang
                    }
                # Request
                result, result_info_dic = self._URL_request(url_forecast, params)

            ########### Source 2 ###########
            elif(source == 'source2'):
                #Params
                #Request
                pass

            # /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ #
            ######################################################

            ####### Request success #######
            if(result == 0): # URL request success
                # Change weather format to standard format
                result_info_dic = source2standard(source, result_info_dic)
                # Saves extra info in the file
                extra_dic = {
                    'lang': lang
                }
                # Saves the content in a local file
                self._save_json(result_info_dic)
                break # Stops the URL requests list loop


        return result, result_info_dic

    def manage_weather(self, goal_vec):
        """
        Manager of the weather class. It updates the result and result_info.
        Examples:
        madrid/forecast_type/tomorrow/basic/101
        sydney/current/0/is_day/011

        @param goal_vec: Vector of the goal.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        if(len(goal_vec) >= self._GOAL_MAX_SIZE-1): # Check if all min fields are completed

            ############## Check weather ################
            location = goal_vec[0] # City name
            forecast_type = goal_vec[1] # Forecast or current info
            date = goal_vec[2] # Date
            info_required = goal_vec[3] # Info wanted

            result, result_info_dic = self._get_info(location, forecast_type, date, info_required)
            
            return result, result_info_dic

        else:
            print("Goal size not completed")
            self._result.result = -1 # Fail
            self._result_info_dic = {}
            return -1, {}


if __name__ == '__main__':
    try:
    	print("[" + pkg_name + "] __main__")
        rospy.init_node('my_node', log_level=rospy.DEBUG)
        weather = Weather()
        result, result_info = weather.manage_weather(['Madrid', 'current', '1', 'basic'])
        print(result_info)


    except rospy.ROSInterruptException:
        pass

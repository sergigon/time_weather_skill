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
import copy
import rospkg

from time_weather_skill.sys_operations import SysOperations
from time_weather_skill.general_functions import *
from time_weather_skill.weather_format_changer import source2standard
from time_weather_skill.datetime_manager import DatetimeManager
from time_weather_skill.timeClass import TimeAstral
from time_weather_skill.csv_reader import *

# Exceptions
class InvalidKey(Exception):
    pass
class GeneralError(Exception):
    pass

class Weather():

    """
    Weather class.
    """

    # Constants
    _GOAL_MAX_SIZE = 5 # Max size of the weather goal
    _WEATHER_FILENAME = 'weather' # Name of the file to store the weather data
    _PARAMS_FILENAME = 'weather_sources_params' # Name of the file to store the sources params data
    _COUNTRY_CODES_FILENAME = 'wikipedia-iso-country-codes' # Name of the file to store the country codes data

    # Lists
    _SOURCE_LIST = ['apixu', 'openweathermap', 'source1', 'source2'] # List of the sources
    _SOURCE_LIST = ['openweathermap', 'apixu', 'source1', 'source2'] # List of the sources
    #_SOURCE_LIST = ['source2'] # List of the sources
    _UPDATE_HOURS = [23, 19, 14, 9, 4] # List of hours for current request
    _INFO_BASIC_LIST = {
        'current': ['date', 'temp_c', 'is_day', 'text', 'code', 'city_name', 'country_name', 'last_updated', 'icon', 'source'], # Basic current list
        'forecast': ['date', 'avgtemp_c', 'text', 'code', 'city_name', 'country_name', 'last_updated', 'icon', 'source'] # Basic advanced list
        }
    _INFO_ADVANCED_LIST = copy.deepcopy(_INFO_BASIC_LIST)
    _INFO_ADVANCED_LIST['current'].extend(['precip_mm']) # Advanced current list
    _INFO_ADVANCED_LIST['forecast'].extend(['mintemp_c', 'maxtemp_c', 'totalprecip_mm']) # Advanced forecast list

    _TIMECLASS_LIST = ['is_day'] # List of the TimeClass parameters

    def __init__(self, rootpath):
        """
        Init method.

        @param rootpath: Root path from where to work with data. Usually, the package path.
        """
        
        # SysOperations object
        self._json_manager = SysOperations()

        # Gets paths
        self._root_path = rootpath # Root path

    def _save_json(self, path, input_dic, extra_dic={}):
        """
        Save weather dictionary in a Json file.

        It can be added extra information.
        It gets the filename autmatically using the dictionary content.
        IMPORTANT: Json data must be in STANDARD format.
        
        @param path: Full directory path.
        @param input_dic: Weather dictionary in STANDARD format.
        @param extra_dic: Dictionary with extra info to save.
        """
        
        # Define filename
        filename = self._WEATHER_FILENAME + '_' + input_dic['common']['city_name'].lower().replace(' ', '-')  + '_' +  input_dic['common']['country_name'].lower().replace(' ', '-') 
        filepath = path + filename + '.json'

        # Add extra info
        input_dic.update(extra_dic)
 
        # Write the data in the JSON file
        print('Saving weather (' + filename + ')')
        self._json_manager.write_json(filepath, input_dic) # Write weather info into JSON file

    def _URL_request(self, url, params):
        """
        URL request method.
		
		@param url: String with the url.
        @param params: Dictionary with the url request params.
        
        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Make URL request
        try:
            # Get the data from the URL in JSON format
            r = requests.get(url, params = params)
            # Error requests
            ##### OpenWeatherMap #####
            if 'cod' in r.json():
                if (r.json()['cod'] == 401): # Invalid Key
                    raise InvalidKey(r.json()['message'])

            ##### Apixu #####
            if 'error' in r.json(): # ERROR in the request
                if (r.json()['error']['code'] == 2006): # Invalid key
                    raise InvalidKey(r.json()['error']['message'])
                else: # General error
                    raise GeneralError(r.json()['error']['message'])

            ### NO errors in the request ###
            print('URL request succeded')
            return 0, r.json()

        except InvalidKey as e:
            rospy.logwarn('[weatherClass] URL request ERROR: Invalid Key (' + str(e) + ')')
            return -1, {}

        except requests.exceptions.ConnectionError as e:
            rospy.logwarn("[weatherClass] URL request ERROR: Connection ERROR: " + str(e))
            return -1, {}

        except GeneralError as e:
            rospy.logwarn("[weatherClass] URL request ERROR: General ERROR: " + str(e))
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
        filename = self._WEATHER_FILENAME
        filename = filename + '_' + city_name.lower().replace(' ', '-') 
        if (country_name != ''): # Country specified
            filename = filename + '_' + country_name.lower().replace(' ', '-') 

        # Search json files in the data folder
        list_json = self._json_manager.ls_json(self._root_path)

        # Search file in the list
        filename_len = len(filename)
        for list_i in list_json:
            if(filename == list_i[:filename_len]):
                filename = list_i[:-5] # Get filename without the '.json' string
                filepath = self._root_path + filename + '.json'
                # Check if JSON files exists
                result_info_dic = self._json_manager.load_json(filepath) # Load json file
                found = True
                break

        if(found and result_info_dic != -1):
            print('\'' + filename + '\'' + ' file found')
            result = 0
        else:
            result, result_info_dic = -1, {}
            rospy.logwarn("Local request ERROR: Local file not found")

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

    def _request_weather(self, location, forecast_type, date):
        """
        Get weather from local or URL sources, and save it if it success.

        @param location: City to get weather. Format: 'Madrid' or 'Madrid, ES'.
        @param forecast_type: 'forecast' or 'current'.
        @param date: Date for the weather.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Initialize variables
        lang = 'es'
        city_name, country_name = location_divider(location) # Divide the location into city and country names
        # Transforms the country name into country code
        filepath = self._root_path + self._COUNTRY_CODES_FILENAME + '.csv'
        if(country_name != ''):
	        country_code = csv_reader_country_codes(filepath, 'English short name lower case', country_name, 'Alpha-2 code')
	        country_name = country_code if (country_code != -1) else country_name

        ################### Make local request ###################
        print("-- Making local request: " + city_name + ", " + country_name + ' --')
        local_result, local_result_info_dic = self._local_request(city_name, country_name)

        # ############### Local request success ################ #
        #if(local_result == 0):
        if(False):
            try:
                # ######### Check if local data is updated ######### #
                # Manage last_update variable datatime
                last_update = DatetimeManager(local_result_info_dic[forecast_type]['last_updated'])
                print('Last update (' + forecast_type + '): ' + last_update.date())
                # Manage current datatime
                now = datetime.datetime.now() # Get current date
                print('Local date: ' + str(now.year) + '-' + str(now.month) + '-' + str(now.day))
                updated = False
                if(now.year == int(last_update.year()) and now.month == int(last_update.month()) and now.day == int(last_update.day())): # Date updated
                    print('Date is updated')
                    if(forecast_type == 'forecast'):
                        pass

                    if(forecast_type == 'current'):
                        pass
                else: # Date NOT updated
                    rospy.logwarn("Weather request ERROR: Local file not updated")
                if(updated):
                    return local_result, local_result_info_dic
                
            except KeyError:
                rospy.logwarn("Weather request ERROR: Key Error")

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

        ##########################################################


        ################## Make URL requests #####################
        url_result, url_result_info_dic = -1, {}
        params_filepath = self._root_path + self._PARAMS_FILENAME + '.csv'
        # Selects the source from the source list
        for source in self._SOURCE_LIST:
            print("-- Making URL request to: '" + source + "' --")

            ############ Get params ############
            print(">> Getting params")
            url, params, extra_info = csv_reader_params(params_filepath, source, forecast_type)
            # Source not found in the csv
            if(url == -1):
                rospy.logwarn("Request Weather ERROR: No params found")
                continue
            ## Checks the limit forecast days ##
            if(forecast_type == 'forecast' and (date < 0 or date >= int(extra_info['limit_forecast_days']))):
                rospy.logwarn("Request Weather ERROR: Forecast day out of range (" + source + ": " + extra_info['limit_forecast_days'] + " limit forecast days), request denied")
                continue # Jumps to the next source

            ########### Update params ##########
            # If the param value starts with   #
            # '--' the value is turned into    #
            # the one specified in the code    #
            ####################################
            for key in params:
                if(params[key][:2] == '__'):
                    if(params[key][2:] == 'location'):
                        params.update({key: location})
                    elif(params[key][2:] == 'lang'):
                        params.update({key: lang})

            ############# Request ##############
            url_result, url_result_info_dic = self._URL_request(url, params)
            rospy.logdebug('[weatherClass] url_result_info_dic: ' + str(url_result_info_dic))

            ####### Request success #######
            if(url_result == 0): # URL request success
                # Change weather format to standard format
                print('Conversion \'' + source + '\' dictionary to standard')
                url_result_info_dic = source2standard(source, forecast_type, url_result_info_dic)
                # Checks if the conversion has failed
                if(url_result_info_dic == -1):
                    rospy.logwarn('Request Weather ERROR: ' + 'Source \'' + source + '\' to standard conversion failed')
                    continue

                # Updates the local dictionary if it exists (if it does not exist, 'local_result_info_dic' will be empty, so no problem)
                # Update local dic variable with new content
                if 'current' in url_result_info_dic:
                    local_result_info_dic['current'] = url_result_info_dic['current']
                if 'forecast' in url_result_info_dic:
                    local_result_info_dic['forecast'] = url_result_info_dic['forecast']
                local_result_info_dic['common'] = url_result_info_dic['common']
                url_result_info_dic = local_result_info_dic # Change the name of the variable

                ##### Save info #####
                # Saves extra info in the file
                extra_dic = {
                    'lang': lang
                }
                # Saves the content in a local file
                self._save_json(self._root_path, url_result_info_dic, extra_dic)
                return url_result, url_result_info_dic

        return -1, {}

    def _get_info(self, location, forecast_type, date, info_required):
        """
        Get the info specified. It uses local methods or make requests to that end.

        @param location: City to get weather. Format: 'Madrid' or 'Madrid, Spain'.
        @param forecast_type: 'forecast' or 'current'.
        @param date: Date for the weather.
        @param info_required: Type of info_required needed.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        print('################## Getting info: ##################')
        print('>> ' + '/' + location + '/' + forecast_type + '/' +  date + '/' + info_required + '/')
        print('###################################################')

        # Initialize result
        result, result_info_dic = -1, {}
        # Initialize variables
        standard_weather_dic = {}

        # Converts to int the date for the result_info dictionary
        date = date_text2num(date)

        ####################### Info Required Lists #######################
        # Prepare info_required list
        info_required = info_required.replace(' ','') # Remove spaces from the string
        info_required_list = info_required.split(",") # Separate the goal in various parts
        # Add new lists
        if('advanced' in info_required_list): # Check if advanced list is requested
            info_required_list.remove('advanced')
            info_required_list.extend(self._INFO_ADVANCED_LIST[forecast_type][:]) # Replace the list with the advanced list
        if('basic' in info_required_list): # Check if basic list is requested
            info_required_list.remove('basic')
            info_required_list.extend(self._INFO_BASIC_LIST[forecast_type][:]) # Replace the list with the basic list
        ###################################################################

        ################# Fill the result_info dictionary #################
        request = False # Indicates if a request has been done

        # Search all the info required
        for info_required_i in info_required_list:
            found = -1 # Indicates if info required has been found before making requests

            # ################ Take data from TimeAstral ################ #
            if(forecast_type == 'current' and info_required_i in self._TIMECLASS_LIST): # This info can be taken with TimeClass
                city_name, _ = location_divider(location) # # Get the city name from location variable
                print('Searching \'' + info_required_i + '\' in TimeAstral: ' + city_name)
                time_var = TimeAstral() # TimeAstral variable
                found, data_time = time_var.get_info(city_name, info_required_i) # Get TimeAstral info
                if(found != -1):
                    result_info_dic.update({info_required_i: data_time})
                else:
                    rospy.logwarn("Weather Class ERROR: Data not found, making a request to get it")


            # ########## Take data from local or URL requests ########### #
            if(found == -1): # Parameter not found with another source
                # Make only ONE request
                if(not request): # A request has not been done yet
                    result, standard_weather_dic = self._request_weather(location, forecast_type, date) # Get the weather info
                    # If there is a request error, it gets out
                    if(result == -1):
                        return result, {}
                    request = True

                # ##### Search the info in the standard_weather_dic ##### #
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
        ###################################################################

        return 0, result_info_dic

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
    	print("[weatherClass]: __main__")
        rospy.init_node('my_node', log_level=rospy.DEBUG)

        # Get path
        rospack = rospkg.RosPack()
        pkg_name = "time_weather_skill"
        pkg_path = rospack.get_path(pkg_name) # Package path
        data_path = pkg_path + '/data/' # Data path

        weather = Weather(data_path)
        result, result_info = weather.manage_weather(['london', 'forecast', 'tomorrow', 'basic'])
        print('#######################')
        print('result_info: ' + str(result_info))
        print('result: ' + str(result))


    except rospy.ROSInterruptException:
        pass

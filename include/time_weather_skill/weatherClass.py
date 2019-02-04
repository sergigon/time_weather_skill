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
import pytz
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
class InvalidURL(Exception):
    pass
class CityNotFound(Exception):
    pass
class NoParam(Exception):
    pass
class GeneralError(Exception):
    pass
class LimitCalls(Exception):
    pass

class Weather():

    """
    Weather class.
    """

    # Params constants
    _LOCAL_COUNTRY_PARAM = 'context/robot/country' # Country code (ISO 3166, 2)
    _LANGUAGE_PARAM = 'context/user/language'

    # Filenames
    _CACHE_FILENAME_STR = 'cache' # Part name of the file to store the weather data (without extension)
    _URL_PARAMS_FILENAME = 'weather_sources_params.csv' # Name of the file to store the sources params data
    _COUNTRY_CODES_FILENAME = 'wikipedia-iso-country-codes.csv' # Name of the file to store the country codes data

    # Lists
    _SOURCE_LIST = ['apixu', 'openweathermap', 'apixu'] # List of the sources
    _UPDATE_HOURS = [23, 19, 14, 9, 4] # List of hours for current request
    _INFO_BASIC_LIST = {
        'current': ['date', 'temp_c', 'is_day', 'text', 'code', 'city_name', 'country_code', 'last_updated', 'icon', 'source'], # Basic current list
        'forecast': ['date', 'avgtemp_c', 'text', 'code', 'city_name', 'country_code', 'last_updated', 'icon', 'source', 'mintemp_c', 'maxtemp_c'] # Basic advanced list
        }
    _INFO_ADVANCED_LIST = copy.deepcopy(_INFO_BASIC_LIST)
    _INFO_ADVANCED_LIST['current'].extend(['precip_mm']) # Advanced current list
    _INFO_ADVANCED_LIST['forecast'].extend(['totalprecip_mm']) # Advanced forecast list

    _TIMECLASS_LIST = ['is_day'] # List of the TimeClass parameters

    def __init__(self, datapath):
        """
        Init method.

        @param datapath: Data path from where to work with data. Usually, the package path.
        """
        
        # SysOperations object
        self._file_manager = SysOperations()

        # Gets paths
        self._data_path = datapath # Data path
        self._cache_path = self._data_path + 'cache/' # Data path
        self._params_path = self._data_path + 'params/' # Params path

    def _update_cache(self, path, input_dic, extra_dic={}):
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
        filename = self._CACHE_FILENAME_STR + '_' + input_dic['common']['city_name'].lower().replace(' ', '-')  + '_' +  input_dic['common']['country_code'].lower().replace(' ', '-') 
        filepath = path + filename + '.json'

        # Add extra info
        input_dic.update(extra_dic)
 
        # Write the data in the JSON file
        print('Saving weather (' + filename + ')')
        self._file_manager.write_json(filepath, input_dic) # Write weather info into JSON file

    def _url_request(self, source, url, params):
        """
        URL request method.
		
		@param url: String with the url.
        @param params: Dictionary with the url request params.
        
        @return result: Final result.
            - -1: Fail
            - -2: Connection Error
            -  1: City not found
            -  0: Url request success

        @return result_info_dic: Weather dictionary result.
        """

        # Make URL request
        try:
            # Get the data from the URL in JSON format
            r = requests.get(url, params = params, timeout=5.05)
            # Error requests
            ##### OpenWeatherMap #####
            if(source == 'openweathermap'): # HTTP Status Code + OpenWeatherMap codes
                rospy.logwarn('cod: ' + str(r.json()['cod']))
                if (str(r.json()['cod']) == '401'): # Invalid Key
                    raise InvalidKey(r.json()['message'])
                elif (str(r.json()['cod']) == '404'): # City Not Found
                    raise CityNotFound(r.json()['message'])
                elif (str(r.json()['cod']) == '429'): # Limit of Calls Exceeded
                    raise LimitCalls(r.json()['message'])
                elif (str(r.json()['cod'])[0] != '2'): # No Success Code (2xx)
                    if 'message' in r.json():
                        raise GeneralError(r.json()['message'])
                    else:
                        raise GeneralError()
            ##### Apixu #####
            if(source == 'apixu'): # https://www.apixu.com/doc/errors.aspx
                if 'error' in r.json():
                    rospy.logwarn('error: ' + str(r.json()['error']))
                    if (r.json()['error']['code'] == 1002 # Key Not Provided
                        or r.json()['error']['code'] == 1003): # Location Not Provided
                        raise NoParam(r.json()['error']['message'])
                    elif (r.json()['error']['code'] == 1005): # Invalid URL
                        raise InvalidURL(r.json()['error']['message'])
                    elif (r.json()['error']['code'] == 1006): # City Not Found
                        raise CityNotFound(r.json()['error']['message'])
                    elif (r.json()['error']['code'] == 2006 # Invalid Key
                        or r.json()['error']['code'] == 2008): # Disabled key
                        raise InvalidKey(r.json()['error']['message'])
                    elif (r.json()['error']['code'] == 2007): # Limit of Calls Exceeded
                        raise LimitCalls(r.json()['error']['message'])
                    else: # General error
                        raise GeneralError(r.json()['error']['message'])

            ### NO errors in the request ###
            print('URL request succeded')
            return 0, r.json()

        # Exceptions
        # Fail #
        except NoParam as e: # Param Not Provided
            rospy.logwarn('[weatherClass] URL request ERROR: Param not provided (%s)' % e)
            return -1, {}
        except InvalidURL as e:
            rospy.logwarn('[weatherClass] URL request ERROR: Invalid URL (%s)' % e)
            return -1, {}
        except InvalidKey as e: # Invalid Key
            rospy.logwarn('[weatherClass] URL request ERROR: Invalid Key (%s)' % e)
            return -1, {}
        except requests.exceptions.MissingSchema as e: # Invalid URL
            rospy.logerr("MissingSchema; %s" % e)
            return -1, {}
        except GeneralError as e: # General Error
            rospy.logwarn('[weatherClass] URL request ERROR: General ERROR (%s)' % e)
            return -1, {}
        except LimitCalls as e: # Limit of Calls Exceeded
            rospy.logwarn('[weatherClass] URL request ERROR: Limit of calls exceded (%s)' % e)
            return -1, {}
        # Connection error #
        except requests.exceptions.ConnectionError as e: # Connection Error
            rospy.logwarn('[weatherClass] URL request ERROR: Connection ERROR (%s)' % e)
            return -2, {}
        except requests.ReadTimeout as e: # Timeout
            rospy.logwarn('[weatherClass] URL request ERROR: Timeout (%s)' % e)
            return -2, {}
        # City not found #
        except CityNotFound as e: # City not found
            rospy.logwarn('[weatherClass] URL request ERROR: City not found (%s)' % e)
            return 1, {}

    def _json_searcher(self, list_json, name):
        """
        Method to search a json. It searchs the first characters of the filenames.

        @param list_json: List of json filenames.
        @param name: Name to search.

        @return filename: Filename found. If not found, it returns -1.
        """

        name_len = len(name)
        # Search file in the list
        for list_i in list_json:
            # There is a coincidence
            if(name == list_i[:name_len]): # Search the first characters in the filename
                return list_i # Get filename
                break

        return -1

    def _local_request(self, city_name, country_code = ''):
        """
        Request local method. Search by the city and country names.

        @param city_name: City to get weather.
        @param country_code: Country code to get weather.

        @return result: Final result.
        @return result_info_dic: Weather dictionary result.
        """

        # Initialize variables
        result, result_info_dic = -1, {}
        found = False
        local_country_code = rospy.get_param(self._LOCAL_COUNTRY_PARAM) # Get local country name

        # Json files list in the data folder
        list_json = self._file_manager.ls_json(self._cache_path)

        # File to search creation (Format ex: 'cache_madrid_es.json')
        if (country_code != ''): # Country specified
            cache_name = self._CACHE_FILENAME_STR + '_' + city_name.lower().replace(' ', '-') + '_' + country_code.lower().replace(' ', '-') # cache_city_country
            rospy.logdebug('Country specified: Searching %s' % cache_name)
            cache_filename = self._json_searcher(list_json, cache_name)
        else: # Country NOT specified
            cache_name = self._CACHE_FILENAME_STR + '_' + city_name.lower().replace(' ', '-') + '_' + local_country_code.lower().replace(' ', '-') # cache_city_local-country
            rospy.logdebug('Country NOT specified: Searching %s (local country)' % cache_name)
            cache_filename = self._json_searcher(list_json, cache_name)
            if (cache_filename == -1):
                # Searchs without country
                cache_name = self._CACHE_FILENAME_STR + '_' + city_name.lower().replace(' ', '-') # cache_city
                rospy.logdebug('Country NOT specified: Searching %s (without country)' % cache_name)
                cache_filename = self._json_searcher(list_json, cache_name)

        if (cache_filename == -1):
            result, result_info_dic = -1, {}
            rospy.logwarn("Local request ERROR: '%s' file not found" % cache_name)
        else:
            cache_filepath = self._cache_path + cache_filename
            # Check if JSON files exists
            result_info_dic = self._file_manager.load_json(cache_filepath) # Load json file
            if(result_info_dic == -1):
                result, result_info_dic = -1, {}
                rospy.logwarn("Local request ERROR: '%s' file not found" % cache_name)
            else:
                print('\'%s\' file found' % cache_name)
                result = 0

        return result, result_info_dic

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
        lang = rospy.get_param(self._LANGUAGE_PARAM)
        local_country_code = rospy.get_param(self._LOCAL_COUNTRY_PARAM) # Get local country name
        city_name, country_code = location_divider(location) # Divide the location into city and country names
        # Transforms the country name into country code
        country_codes_filepath = self._params_path + self._COUNTRY_CODES_FILENAME
        if(country_code != ''): # Country code specified
            # Country name to code conversion, if needed
	        country_code_aux = csv_reader_IO(country_codes_filepath, 'English short name lower case', country_code, 'Alpha-2 code') 
	        country_code = country_code_aux if (country_code_aux != -1) else country_code

        ################### Make local request ###################
        print("##--## Making local request: " + city_name + ", " + country_code + ' ##--##')
        # Local request
        local_result, local_result_info_dic = self._local_request(city_name, country_code)

        ####### Request result management #######
        # -- Request success -- #
        if(local_result == 0):
            try:
                # ####### Check if local data is updated ####### #
                # Manage datatimes
                now_utc = datetime.datetime.now(tz = pytz.utc) # Actual datetime in UTC (+00:00)
                last_updated = datetime.datetime.fromtimestamp(local_result_info_dic[forecast_type]['last_updated'], tz=pytz.utc) # Last updated in UTC (+00:00)
                rospy.logdebug('Current datetime: %s' % now_utc)
                rospy.logdebug('Last updated (%s): %s' % (forecast_type, last_updated))

                # Language check #
                if(local_result_info_dic['lang'].lower() != lang.lower()):
                    raise GeneralError('Language not updated')
                # Timedate check #
                if(forecast_type == 'current'):
                    # Get datetime without min, sec and microsec
                    now_utc = now_utc.replace(minute=0, second=0, microsecond=0) # (hh:00:00)
                    # Update each hour
                    if((now_utc-last_updated).total_seconds()>0):
                        raise GeneralError('Datetime not updated')
                if(forecast_type == 'forecast'):
                    # Get datetime without hour, min, sec and microsec
                    now_utc = now_utc.replace(hour=0, minute=0, second=0, microsecond=0) # (00:00:00)
                    # Update each day
                    if((now_utc-last_updated).total_seconds()>0):
                        raise GeneralError('Datetime not updated')

                # Cache already updated #
                rospy.logdebug('Cache is already updated. Getting local info')
                return local_result, local_result_info_dic

            except KeyError as e:
                rospy.logwarn('[weatherClass] Weather request ERROR: Key Error (%s)' % e)
            except GeneralError as e:
                rospy.logwarn('[weatherClass] Cache must be updated (%s)' % e)
        #########################################
        ##########################################################


        ################## Make URL requests #####################
        url_result, url_result_info_dic = -1, {}
        params_filepath = self._params_path + self._URL_PARAMS_FILENAME # Filepath for url request parameters

        # ################### Source list ################### #
        for source in self._SOURCE_LIST:
            print("##--## Making URL request to: '" + source + "' ##--##")

            ################## Get params ##################
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
            ################################################

            ################### Request ####################
            n_attempts = 1 # Number of attempts for conexion errors
            only_city = False # Variable to set only city location
            new_request = True # Indicate if make new request with same source

            if(country_code == ''): # Country NOT specified
                location_new = location_combiner(city_name, local_country_code)
                rospy.logdebug('Country not specified. Searching with local country (%s)' % location_new)
                only_city = True # If city not found, search without the country
            else: # Country specified
                location_new = location_combiner(city_name, country_code)
                rospy.logdebug('Country specified (%s)' % location_new)

            # ################ Request loop ################# #
            while(new_request == True):
                new_request = False # New source
                ########### Update params ##########
                # If the param value starts with   #
                # '__' the value is turned into    #
                # the one specified in the code    #
                ####################################
                for key in params:
                    if(params[key][:2] == '__'):
                        if(params[key][2:] == 'location'):
                            params.update({key: location_new})
                        elif(params[key][2:] == 'lang'):
                            params.update({key: lang})
                # URL request
                url_result, url_result_info_dic = self._url_request(source, url, params)
                rospy.logdebug('[weatherClass] url_result_info_dic: ' + str(url_result_info_dic))
                ################################################

                ####### Request result management #######
                ############################
                # -1: Fail                 #
                # -2: Connection Error     #
                #  1: City not found       #
                #  0: Url request success  #
                ############################
                # -- Request success -- #
                if(url_result == 0):
                    rospy.logdebug('Request success')
                    # Change weather format to standard format
                    print('Conversion \'' + source + '\' dictionary to standard')
                    url_result_info_dic = source2standard(source, forecast_type, url_result_info_dic)
                    # Checks if the conversion has failed
                    if(url_result_info_dic == -1):
                        rospy.logwarn('Request Weather ERROR: ' + 'Source \'' + source + '\' to standard conversion failed')
                    else:
                        ########### Updates cache ##########
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
                        self._update_cache(self._cache_path, url_result_info_dic, extra_dic)
                        ####################################
                        return url_result, url_result_info_dic

                # -- Request Fail -- #
                elif(url_result == -1):
                    rospy.logerr('Request Weather ERROR: Request fail')

                # -- Connection Error -- #
                elif(url_result == -2):
                    rospy.logwarn('Request Weather ERROR: Connection Error (attempt %s)' % n_attempts)
                    if(n_attempts<=2):
                        n_attempts+=1
                        new_request = True # Request again
                    else:
                        rospy.logwarn('Request Weather ERROR: Connection Error. Exceeded attemps limit')

                # -- City not found -- #
                elif(url_result == 1):
                    rospy.logwarn('Request Weather ERROR: City Not Found Error (%s)' % location_new)
                    if(only_city): # Location without country asked
                        location_new = city_name # Set location without the country
                        rospy.logdebug('Searching location without country (%s)' % location_new)
                        only_city = False # Set variable to not make another request
                        new_request = True # Request again

            # ############################################### #
            rospy.logwarn('Searching next source')

        # ################################################### #
        # No more sources in the list
        rospy.logerr('[weatherClass] No more sources')
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

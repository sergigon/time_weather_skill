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


class DatetimeManager():
    """
    Datetime manager class.
    """

    def __init__(self, datetime = -1):
        """
        Init method
        """

        # Initialize variables
        self._date, self._year, self._month, self._day = '', '', '', ''
        self._time, self._hour, self._min, self._sec = '', '', '', ''

        if(datetime != -1):
            self.manage_datetime(datetime)



    def manage_datetime(self, datetime):
        """
        Manage the given datetime and separate it in different atributes.

        @param datetime: Date and/or time to be managed.
        Types of inputs:
        + Date and time: '2018-11-08 17:45'
        + Date: '2018-11-08'
        + Time: '17:45'
        """

        # Initialize variables
        self._date, self._year, self._month, self._day = '', '', '', ''
        self._time, self._hour, self._min, self._sec = '', '', '', ''
        date_check, time_check = False, False

        # Checks if exists date and time
        if('-' in datetime): # Check if there is date
            date_check = True
        if(':' in datetime): # Check if there is time
            time_check = True
        if(date_check and time_check):
            self._date, self._time = datetime.split(' ')
        elif(date_check):
            self._date = datetime
        elif(time_check):
            self._time = datetime

        # Fills data
        if(self._date != ''):
            self._year, self._month, self._day = self._date.split('-')
        if(self._time != ''):
            aux_vec = self._time.split(':')
            if(len(aux_vec) == 2):
                self._hour, self._min = aux_vec[0], aux_vec[1]
            if(len(aux_vec) == 3):
                self._hour, self._min, self._sec = aux_vec[0], aux_vec[1], aux_vec[2]

    def date(self):
        return self._date

    def year(self):
        return self._year

    def year_int(self):
        if(self._year == ''):
            return -1
        else:
            return int(self._year)

    def month(self):
        return self._month

    def month_int(self):
        if(self._month == ''):
            return -1
        else:
            return int(self._month)

    def day(self):
        return self._day

    def day_int(self):
        if(self._day == ''):
            return -1
        else:
            return int(self._day)

    def time(self):
        return self._time

    def hour(self):
        return self._hour

    def hour_int(self):
        if(self._hour == ''):
            return -1
        else:
            return int(self._hour)

    def min(self):
        return self._min

    def min_int(self):
        if(self._min == ''):
            return -1
        else:
            return int(self._min)

    def sec(self):
        return self._sec

    def sec_int(self):
        if(self._sec == ''):
            return -1
        else:
            return int(self._sec)

if __name__ == '__main__':

    datetime = '2018-11-08 17:45'
    date = '2020-12-20'
    time = '20:00'
    datetime = DatetimeManager(datetime)
    print(datetime.date())
    print(datetime.year(), datetime.year_int())
    print(datetime.month(), datetime.month_int())
    print(datetime.day(), datetime.day_int())

    datetime.manage_datetime(time)
    print(datetime.time())
    print(datetime.hour(), datetime.hour_int())
    print(datetime.min(), datetime.min_int())

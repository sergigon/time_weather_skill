import requests # URL requests
import commands # Comandos de terminal (http://www.rafalinux.com/?p=1613)

# Constant names
apixu_limit_weather = 7 # 7 days limit
apixu_month_limit_calls = 10000 # 10000 limit calls per month

# Variable names
api_key = '9838870ce6324daf95d161656180811' # sergigon@ing.uc3m.es
location = "Madrid, Spain" # Se puede especificar pais o no

# URLs
url_current = 'http://api.apixu.com/v1/current.json' # URL para current weather
url_forecast = 'http://api.apixu.com/v1/forecast.json' # URL para forecast weather

url_icon = 'http://www.apixu.com/doc/Apixu_weather_conditions.json' # URL para informacion de los iconos

# Parametros para hacer el URL request
params = {
	'key': api_key,
	'q': location,
	'days': apixu_limit_weather, # Only for forecast
	'lang': 'es' # Espanol
}

# Hago el URL request para el weather
r = requests.get(url_forecast, params = params)
data = r.json()

# Que tiempo hara la semana que viene
forecast_day = 6
#print(data['forecast']['forecastday'][forecast_day])

print("El proximo dia " + data['forecast']['forecastday'][forecast_day]['date'] + " ")


######## Showing Icon ##########
icon_code = '' # Codigo del icono
icon_number = '' # Numero de la imagen del icono
day = 'day' # Indica dia o noche

print(data['forecast']['forecastday'][forecast_day]['day']['condition']['text'])
print(data['forecast']['forecastday'][forecast_day]['day']['condition']['code'])
icon_code = data['forecast']['forecastday'][forecast_day]['day']['condition']['code'] # Guardo el codigo del icono

# Pido informacion sobre los codigos y numeros de los iconos
i = requests.get(url_icon)
icons_data = i.json()

for icon in icons_data:
	if (icon['code'] == icon_code):
		icon_number = str(icon['icon'])
		break

command = 'fim -a weather_icons/64x64/' + day + '/' + icon_number + '.png'

res = commands.getstatusoutput(command)
if res[0] == 0:
	print res[1]
else:
	print "Error: "+ str(res[0])
	print "Descripcion: " + res[1]

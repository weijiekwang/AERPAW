import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from geopy import distance

f_search = "ROVER_SEARCH_DATA_2023-07-31_00%3A55%3A10.csv"
#f_gps = "2023-07-12_23_15_13_vehicleOut.txt"


df =  pd.read_csv(f_search)
df['timestamp'] = pd.to_datetime(df['timestamp'])
#df.info()


#df_rover = pd.read_csv(f_gps, header=None).tail(1)
#rover_pos = np.array([df_rover[[1]], df_rover[[2]]]).reshape((1,2))
#df = df.assign(dist = np.sqrt( (rover_pos[0, 0] - df.longitude.values)**2 +  (rover_pos[0, 1] - df.latitude.values)**2 ) )


#df.plot(x = 'longitude', y = 'latitude', c = 'RSSI', cmap = 'cool', s = 100, kind='scatter')
plt.scatter( x = df['longitude'], y = df['latitude'], c=df['RSSI'], cmap='cool', s=70)


plt.xlabel("Longitude", fontdict=None, labelpad=None, loc=None)
plt.ylabel("Latitude", fontdict=None, labelpad=None, loc=None)
plt.title("Signal Power with Different Locations")

plt.show()

df.plot( x = 'timestamp', y = 'dist', c = 'RSSI', cmap = 'cool',  kind = 'scatter')
plt.yscale('log')
plt.xlabel("(Minutes)")

plt.title("Signal Power Over Time")
plt.show()
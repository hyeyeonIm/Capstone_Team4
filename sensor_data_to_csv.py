import pandas as pd
import pymysql
from datetime import datetime

# print('start-time : ', str(datetime.now())[:19])
address = '/home/pi2/Documents/csv_files/'
file_name = 'DHT_measures_'
now = datetime.now()
file_time = now.strftime('%Y-%m-%d')

conn=pymysql.connect(host="localhost",
                     user="sensor_user",
                     passwd="987654321",
                     db="sensor_data")

query = 'SELECT * FROM collect_data'
df = pd.read_sql_query(query, conn)

# path_or_buf = '/home/pi2/Documents/csv_files/DHT_measures_2024-03-01.csv'
df.to_csv(path_or_buf=address+file_name+file_time+'.csv', index=False)

# print('end-time : ', str(datetime.now())[:19])
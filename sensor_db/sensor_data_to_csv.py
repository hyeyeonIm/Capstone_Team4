import pandas as pd
import pymysql
from datetime import datetime

# print('start-time : ', str(datetime.now())[:19])
# This is for Raspberry Pi address
# address = '/home/pi2/Documents/csv_files/'
# This is for matt Ubuntu
# address = '/home/matt/work/csv_files/'
# This is for haley Ubuntu
address = '/home/haley/catkin_ws/roomInfo/'

file_name = 'DHT_measures_'
now = datetime.now()
file_time = now.strftime('%Y-%m-%d')

# Change to fit your database
conn=pymysql.connect(host="localhost",
                     user="sensor_user",
                     passwd="987654321",
                     db="sensor_data")

query = 'SELECT * FROM collect_data'
df = pd.read_sql_query(query, conn)

# example of file name
# path_or_buf = '/home/pi2/Documents/csv_files/DHT_measures_2024-03-01.csv'
df.to_csv(path_or_buf=address+file_name+file_time+'.csv', index=False)

# print('end-time : ', str(datetime.now())[:19])

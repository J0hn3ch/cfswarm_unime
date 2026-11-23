import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

token = os.environ.get("INFLUXDB_TOKEN")
org = "universome"
url = "http://192.168.137.4:8086"

influx_client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)

bucket="crazyflie_monitor"

write_api = influx_client.write_api(write_options=SYNCHRONOUS)

for value in range(5):
  point = (
    Point("measurement1")
    .tag("tagname1", "tagvalue1")
    .field("field1", value)
  )
  write_api.write(bucket=bucket, org="universome", record=point)
  time.sleep(1) # separate points by 1 second

query_api = influx_client.query_api()

query = """from(bucket: "crazyflie_monitor")
 |> range(start: -10m)
 |> filter(fn: (r) => r._measurement == "measurement1")"""

tables = query_api.query(query, org="universome")

for table in tables:
  for record in table.records:
    print(record)


# use the mean() function to calculate the average value of data points in the last 10 minutes
query = """from(bucket: "crazyflie_monitor")
  |> range(start: -10m)
  |> filter(fn: (r) => r._measurement == "measurement1")
  |> mean()"""
tables = query_api.query(query, org="universome")

for table in tables:
  for record in table.records:
    print(record)

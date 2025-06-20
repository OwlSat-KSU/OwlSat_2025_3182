from datetime import datetime
import pytz

# Assume current local time (e.g., US Eastern)
local_tz = pytz.timezone("US/Eastern")
local_time = local_tz.localize(datetime.now())

# Convert to UTC
utc_time = local_time.astimezone(pytz.utc)

print("Local time:", local_time)
print("Converted to UTC:", utc_time)

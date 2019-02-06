import subprocess
import os
import time

flag=0
with open("temp_data/data.csv") as f:
    first=f.read(1)
    if not first:
        flag=1

if flag==1:
    with open("temp_data/data.csv","w") as f:
        f.write("Time,Temperature\n")



while True:
    output=subprocess.check_output(["/opt/vc/bin/vcgencmd","measure_temp"])
    value=float(output.split("=")[1].split("'")[0])

    with open("temp_data/data.csv","a+") as f:
        f.write(str(int(time.time()))+","+str(value)+"\n")
    time.sleep(0.5)
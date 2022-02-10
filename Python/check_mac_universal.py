import setup

import subprocess
#print results of lipo
for l in setup.full_libs:
    if not l.endswith('.platform'):
        try:
            print(subprocess.check_output(["lipo","-info",l]).decode('utf-8').strip())
        except subprocess.CalledProcessError as e:
            print("Error while checking",l)
            print(e)

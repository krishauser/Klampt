import subprocess

libline = None
with open('setup.py','r') as f:
    for line in f.readlines():
        if line.startswith('libs = '):
            libline = line
            break

libs = libline.split('"""')[1].split(';')
print(libs)


#print results of lipo
for l in libs:
    if not l.endswith('.framework'):
        try:
            print(subprocess.check_output(["lipo","-info",l]).decode('utf-8').strip())
        except subprocess.CalledProcessError as e:
            print("Error while checking",l)
            print(e)

import os
import glob
import sys

root_py_folder = "klampt"
if sys.version_info[0] == 2:
    #python2 version has drifted from current Python3 version... maintain separate packages list for compatibility
    root_py_folder = "python2_version/klampt"
    subpackages = ['apps','io','math','model','model/create','plan','plan/kinetrajopt','sim','vis','vis/backends','vis/ipython']
else:
    subpackages = ['apps','control','control/blocks','control/io','io','math','math/autodiff','model','model/create','plan','plan/kinetrajopt','sim','vis','vis/backends','vis/ipython']

#need to grab the existing C extension module .py and .so files
import site
import glob
pip_klampt_version = '0.8.5'
py_version = '%d.%d'%(sys.version_info[0],sys.version_info[1])
klampt_path = os.path.join(site.getsitepackages()[0],'klampt')
if not os.path.exists(klampt_path):
    print("Klampt",pip_klampt_version,"wasn't installed by pip?")
    exit(1)
import shutil
dontcopy = ['robotsim.py','motionplanning.py']
def docopy(fn):
    basefn = fn[len('klampt/'):]
    print("Copying",fn,"to",os.path.join(klampt_path,basefn))
    shutil.copy(fn,os.path.join(klampt_path,basefn))
for path in [root_py_folder] + [os.path.join(root_py_folder,p) for p in subpackages]:
    for fn in glob.glob(os.path.join(path,'*.py')):
        if os.path.basename(fn) not in dontcopy:
            docopy(fn)
for fn in glob.glob(os.path.join(root_py_folder,'data/*.*')):
  if os.path.basename(fn) not in dontcopy:
      docopy(fn)

print("Klampt pip install patching complete")
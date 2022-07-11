import os
import glob
import sys

pip_klampt_version = '0.9.0'
root_py_folder = "klampt"
if sys.version_info[0] == 2:
    #python2 version has drifted from current Python3 version... maintain separate packages list for compatibility
    root_py_folder = "python2_version/klampt"
    subpackages = ['apps','io','math','model','model/create','plan','plan/kinetrajopt','sim','vis','vis/backends','vis/ipython']
else:
    subpackages = ['apps','control','control/blocks','control/io','io','math','math/autodiff','model','model/create','plan','plan/kinetrajopt','sim','vis','vis/backends','vis/ipython']

#find the klampt install path...

klampt_path = None

#method 1: check sites
import site
for path in [site.getusersitepackages()] + site.getsitepackages():
    if os.path.exists(os.path.join(path,'klampt')):
        klampt_path = os.path.join(path,'klampt')
        break
if klampt_path is None:
    #method 2: check distutils
    from distutils.sysconfig import get_python_lib
    site_packages_dir = get_python_lib()
    if os.path.exists(os.path.join(site_packages_dir,'klampt')):
        klampt_path = os.path.join(site_packages_dir,'klampt')
if klampt_path is None:
    #find via import
    oldpath = sys.path[:]  #don't search current working directory
    if "" in sys.path:
        sys.path.remove("");  
    fullfile = os.path.abspath(__file__)
    filepath = os.path.split(fullfile)[0]
    if filepath in sys.path:
        sys.path.remove(filepath)
    try:
        import klampt
        klampt_path = klampt.__path__[0]
    except ImportError as e:
        print("klampt doesn't seem to be installed for this version of python, perhaps wasn't installed by pip?")
        print(e)
        exit(1)
    sys.path = oldpath
    if klampt.__version__ != pip_klampt_version:
        print("Installed klampt version doesn't match current module base version: {} vs {}".format(klampt.__version__,pip_klampt_version))
        exit(1)

print("Installed klampt path:",klampt_path)


#need to keep the existing C extension module .py and .so files
import shutil
dontcopy = ['robotsim.py','motionplanning.py']
ignore_prefix_len = len(root_py_folder)+1
def docopy(fn):
    basefn = fn[ignore_prefix_len:]
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
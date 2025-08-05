"""YCB object downnloader.  Modified from code from Yale University's Grablab.

Copyright notice from the original code:

Copyright 2015 Yale University - Grablab
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:\
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import os
import json
import urllib
import urllib.request
import tarfile
from typing import List,Dict,Union,Any
import klampt

_base_url = "http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/"
_objects_url = _base_url + "objects.json"


def object_list() -> List[str]:
    """Returns a list of all YCB objects."""
    url = _objects_url
    response = urllib.request.urlopen(url)
    data = json.loads(response.read())
    return data["objects"]


def options() -> Dict[str,List[str]]:
    """Returns a list of available options for downloading YCB objects.

    Options for 'scan' include:    
    - 'any' will download the first available scan type.
    - 'berkeley_rgbd' contains all of the depth maps and images from the Carmines.
    - 'berkeley_rgb_highres' contains all of the high-res images from the Canon cameras.
    - 'berkeley_processed' contains all of the segmented point clouds and textured meshes.
    - 'google_16k' contains google meshes with 16k vertices.
    - 'google_64k' contains google meshes with 64k vertices.
    - 'google_512k' contains google meshes with 512k vertices.
    
    See the website for more details.
    """
    return {'scan':["any", "berkeley_rgbd", "berkeley_rgb_highres", "berkeley_processed", "google_16k", "google_64k", "google_512k"],
            'model':['textured','nontextured']}


def preferred_scans() -> List[str]:
    """Returns the list of scans attempted when scan='any' is specified."""
    return ['google_16k', 'google_64k', 'google_512k', 'berkeley_processed']


def download(objects : Union[str,List[str]] = 'all',
             scan : Union[str,List[str]] = 'any',
             model : str = None,
             output_directory = './ycb') -> List[str]:
    """
    Downloads YCB objects and their associated files.

    Parameters:
    - objects: One or more object names to download, or 'all' for all objects.
        E.g., ["002_master_chef_can", "003_cracker_box"]
    - scan: Scan option(s) to download.  The first available scan will be downloaded
        if 'any' is specified.
    - model: ignored. (All models for the given object are downloaded)
    - output_directory: Directory to save the downloaded files.
    
    Returns:
    - List of paths to the downloaded files.
    """
    output_directory = os.path.expanduser(output_directory)
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    if objects == 'all':
        objects = object_list()
    if isinstance(objects, str):
        objects = [objects]
    stop_on_first = False
    if isinstance(scan, str):
        if scan == 'any':
            scan = preferred_scans()
            stop_on_first = True
        else:
            scan = [scan]

    def report_hook(count, block_size, total_size):
        percent = float(count * block_size) / total_size
        print(f"\r   {percent:.2%}", end='')

    files = []
    for obj in objects:
        for scan in scan:
            url = _tgz_url(obj, scan)
            if not _check_url(url):
                print(f"URL {url} does not exist, skipping {obj} {scan}")
                continue
            filename = os.path.join(output_directory, f"{obj}_{scan}.tgz")
            print(f"Downloading {url} to {filename}")
            urllib.request.urlretrieve(url, filename, report_hook)

            result_folder = os.path.join(output_directory, f"{obj}/{scan}")
            with tarfile.open(filename, "r:gz") as tar:
                tar.extractall(path=output_directory)
            os.remove(filename)  # Remove the .tgz file after extraction

            if scan == 'berkeley_processed':
                import shutil
                os.makedirs(result_folder, exist_ok=True)
                shutil.move(os.path.join(output_directory, f"{obj}/clouds"), result_folder)
                shutil.move(os.path.join(output_directory, f"{obj}/tsdf"), result_folder)
                shutil.move(os.path.join(output_directory, f"{obj}/poisson"), result_folder)
            files.append(result_folder)
            
            if stop_on_first:
                break
    return files

def load_files(objects : Union[str,List[str]],
         scan : str = 'any',
         model : str = 'textured',
         download_directory = './ycb') -> List[str]:
    """Loads one or more YCB objects and returns a list of preferred
    file names pointing directly to the "best" geometry files.

    If they have already been downloaded, they will be loaded from the local
    filesystem.  If not, they will be downloaded to the specified directory.
    """
    if objects == 'all':
        objects = object_list()
    if isinstance(objects, str):
        objects = [objects]
    all_options = options()
    models = []
    for obj in objects:
        folder = None
        if scan == 'any':
            for opt in all_options['scan']:
                if os.path.exists(os.path.join(download_directory, f"{obj}/{opt}")):
                    file_type = opt
                    folder = os.path.join(download_directory, f"{obj}/{opt}")
                    break
            if folder is None:
                res = download(objects=[obj], scan=scan, output_directory=download_directory)
                folder = res[0] if res else None
        else:
            folder = os.path.join(download_directory, f"{obj}/{scan}")
            if not os.path.exists(folder):
                print(f"Object {obj} with option {scan} not found in {download_directory}, downloading...")
                res = download(objects=[obj], scan=scan, output_directory=download_directory)
                folder = res[0] if res else None
        if folder is None:
            print(f"Object {obj} with option {scan} could not be downloaded, skipping...")
            continue
        if 'berkeley_processed' in folder:
            #prefer the TSDF processed meshes
            folder = os.path.join(folder, 'tsdf')
        #now load the object
        g = None
        for file in os.listdir(folder): 
            if model=='any' or file.startswith(model):
                if file.endswith('.dae') or file.endswith('.obj') or file.endswith('.stl') or file.endswith('.ply'):
                    filename = os.path.join(folder, file)
                    g = filename
                    break
        if g is None:
            print(f"Could not find a valid geometry file for {obj} in {folder}")
            continue
        models.append(g)
    return models


def load(objects : Union[str,List[str]],
         scan : str = 'any',
         model : str = 'textured',
         download_directory = './ycb') -> Union[klampt.Geometry3D,List[klampt.Geometry3D]]:
    """Loads one or more YCB objects into Klampt Geometry3D objects.

    If they have already been downloaded, they will be loaded from the local
    filesystem.  If not, they will be downloaded to the specified directory.
    """
    files = load_files(objects, scan, model, download_directory)
    models = [klampt.Geometry3D(f) for f in files]
    return models if len(models) > 1 else models[0] if models else None


def _download_file(url, filename):
    u = urllib.request.urlopen(url)
    f = open(filename, 'wb')
    meta = u.info()
    file_size = int(meta.getheaders("Content-Length")[0])
    print("Downloading: %s (%s MB)" % (filename, file_size/1000000.0))

    file_size_dl = 0
    block_sz = 65536
    while True:
        buffer = u.read(block_sz)
        if not buffer:
            break

        file_size_dl += len(buffer)
        f.write(buffer)
        status = r"%10d  [%3.2f%%]" % (file_size_dl/1000000.0, file_size_dl * 100. / file_size)
        status = status + chr(8)*(len(status)+1)
        print(status,end='')
    print("\nDownload complete: %s" % filename)
    f.close()


def _tgz_url(object, type):
    if type in ["berkeley_rgbd", "berkeley_rgb_highres"]:
        return _base_url + "berkeley/{object}/{object}_{type}.tgz".format(object=object,type=type)
    elif type in ["berkeley_processed"]:
        return _base_url + "berkeley/{object}/{object}_berkeley_meshes.tgz".format(object=object,type=type)
    else:
        return _base_url + "google/{object}_{type}.tgz".format(object=object,type=type)

def _check_url(url):
    try:
        request = urllib.request.Request(url)
        request.get_method = lambda : 'HEAD'
        response = urllib.request.urlopen(request)
        return True
    except Exception as e:
        return False



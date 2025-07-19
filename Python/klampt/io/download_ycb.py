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
    
    'berkeley_rgbd' contains all of the depth maps and images from the Carmines.
    'berkeley_rgb_highres' contains all of the high-res images from the Canon cameras.
    'berkeley_processed' contains all of the segmented point clouds and textured meshes.
    'google_16k' contains google meshes with 16k vertices.
    'google_64k' contains google meshes with 64k vertices.
    'google_512k' contains google meshes with 512k vertices.
    
    See the website for more details.
    """
    return {'scan':["berkeley_rgbd", "berkeley_rgb_highres", "berkeley_processed", "google_16k", "google_64k", "google_512k"],
            'model':['textured','nontextured']}


def download(objects : Union[str,List[str]] = 'all',
             scan : Union[str,List[str]] = 'google_16k',
             model : str = None,
             output_directory = './ycb') -> List[str]:
    """
    Downloads YCB objects and their associated files.

    Parameters:
    - objects: One or more object names to download, or 'all' for all objects.
        E.g., ["002_master_chef_can", "003_cracker_box"]
    - scan: Scan option(s) to download.
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
    if isinstance(options, str):
        options = [options]

    def report_hook(count, block_size, total_size):
        percent = float(count * block_size) / total_size
        print(f"\r   {percent:.2%}", end='')

    files = []
    for obj in objects:
        for file_type in options:
            url = _tgz_url(obj, file_type)
            if not _check_url(url):
                print(f"URL {url} does not exist, skipping {obj} {file_type}")
                continue
            filename = os.path.join(output_directory, f"{obj}_{file_type}.tgz")
            print(f"Downloading {url} to {filename}")
            urllib.request.urlretrieve(url, filename, report_hook)

            with tarfile.open(filename, "r:gz") as tar:
                tar.extractall(path=output_directory)
            os.remove(filename)  # Remove the .tgz file after extraction
            result_folder = os.path.join(output_directory, f"{obj}/{file_type}")
            files.append(result_folder)
    return files


def load(objects : Union[str,List[str]],
         scan : str = 'any',
         model : str = 'textured',
         download_directory = './ycb') -> Union[klampt.Geometry3D,List[klampt.Geometry3D]]:
    """Loads one or more YCB objects into Klampt Geometry3D objects.

    If they have already been downloaded, they will be loaded from the local
    filesystem.
    """
    if objects == 'all':
        objects = object_list()
    if isinstance(objects, str):
        objects = [objects]
    all_options = options()
    models = []
    for obj in objects:
        file_type = scan
        if file_type == 'any':
            for opt in all_options['scan']:
                if os.path.exists(os.path.join(download_directory, f"{obj}_{opt}")):
                    file_type = opt
                    break
            if file_type == 'any':
                file_type = 'google_16k'  # Default to google_16k if no other option found
        if not os.path.exists(os.path.join(download_directory, f"{obj}/{file_type}")):
            print(f"Object {obj} with option {file_type} not found in {download_directory}, downloading...")
            download(objects=[obj], options=[file_type], output_directory=download_directory)
        #now load the object
        folder = os.path.join(download_directory, f"{obj}/{file_type}")
        g = None
        for file in os.listdir(folder): 
            if model=='any' or file.startswith(model):
                if file.endswith('.dae') or file.endswith('.obj') or file.endswith('.stl') or file.endswith('.ply'):
                    filename = os.path.join(folder, file)
                    print(f"Loading {filename}")
                    g = klampt.Geometry3D(filename)
                    break
        if g is None:
            print(f"Could not find a valid geometry file for {obj} in {folder}")
            continue
        models.append(g)
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



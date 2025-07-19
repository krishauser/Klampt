"""Objaverse downloader.

Requires objaverse, trimesh, and optional fuzzysearch modules.
"""

import objaverse
import fuzzysearch
import trimesh
from klampt.io.trimesh_convert import from_trimesh
from klampt.math import so3,se3,vectorops
from klampt import Geometry3D
import math
import os
from typing import List, Sequence, Tuple, Dict, Union, Optional, Any

INCHES_TO_METERS = 0.0254
MM_TO_METERS = 0.001
_first_call = True


def set_temp_directory(directory: str):
    """Sets the temporary directory for objaverse downloads."""
    objaverse.BASE_PATH = os.path.expanduser(directory)
    objaverse._VERSIONED_PATH = os.path.join(objaverse.BASE_PATH, "hf-objaverse-v1")
    print("Objaverse temporary directory set to", objaverse.BASE_PATH)
    global _first_call
    _first_call = False

def object_list() -> List[str]:
    """Returns a list of all object annotations."""
    global _first_call
    if _first_call:
        print("Objaverse temporary directory is set to", objaverse.BASE_PATH)
        _first_call = False
    return objaverse.load_uids()

def object_annotations(uids: Optional[List[str]] = None) -> Dict[str, Any]:
    """Returns the metadata for some / all objects in the dataset.
    Args:
        uids: A list of uids with which to load metadata. If None, it loads
        the metadata for all uids.
    Returns:
        A dictionary mapping the uid to the metadata.
    """
    global _first_call
    if _first_call:
        print("Objaverse temporary directory is set to", objaverse.BASE_PATH)
        _first_call = False
    return objaverse.load_annotations(uids)

def object_classes() -> List[str]:
    """Returns a list of all object classes."""
    global _first_call
    if _first_call:
        print("Objaverse temporary directory is set to", objaverse.BASE_PATH)
        _first_call = False
    return list(objaverse.load_lvis_annotations().keys())

def _is_uid(uid: str) -> bool:
    """Checks if the given string is a valid UID with characters 0-9 and a-f."""
    return all(c.isdigit() or c in 'abcdef' for c in uid) and len(uid) == 32

def options() -> Dict[str, List[str]]:
    """Returns a dictionary of available options for objaverse."""
    return {
        'index' : ['random','all',int],
        'flattened' : [True,False],
        'scale' : ['auto',float]
    }


def _fuzzy_match(keys : Sequence[str], query : str, max_l_dist : int) -> Optional[str]:
    """Finds the best match for a query to subkeys of a list of keys. """
    best_match = None
    match_score = float('inf')
    for k in keys:
        res = fuzzysearch.find_near_matches(query, k, max_l_dist = max_l_dist)
        for match in res:
            start_penalty = match.start
            end_penalty = (len(k)-match.end)
            if k[match.start] == ' ' or k[match.start] == '_':
                start_penalty = 1
            if match.end < len(k):
                if k[match.end] == ' ' or k[match.end] == '_':
                    end_penalty = 1
            score = match.dist + start_penalty + end_penalty
            print("Match",k,"distance",match.dist,"range",match.start,match.end,"score",score)
            if score < match_score:
                best_match = k
                match_score = score
    return best_match


def download(uids_or_queries : Union[str,List[str]],
             index : Union[str,int] = 'all',
             flattened = None,
             scale = None,
             output_directory = None) -> List[str]:
    """Downloads one or more objects from objaverse.
    
    Args:
        uids_or_queries: one or more queries or UIDs to download.
            E.g., ['alarm clock','ambulance','apple']
        index: the i'th index in the object list
        flattened: ignored
        scale : ignored
        output_directory: the directory to save the downloaded objects.
    """
    if output_directory is not None:
        set_temp_directory(output_directory)
    print("Downloading Objaverse objects to", objaverse.BASE_PATH)
    if isinstance(uids_or_queries, str):
        uids_or_queries = [uids_or_queries]
    ids = []
    if any(not _is_uid(item) for item in uids_or_queries):
        lvis = objaverse.load_lvis_annotations()
    for item in uids_or_queries:
        if _is_uid(item):
            ids.append(item)
        else:
            if item in lvis:
                item_ids = lvis[item]
            else:
                print("Couldn't find exact match to {}, finding best fuzzy match".format(item))
                match_item = _fuzzy_match(item, lvis.keys(), max_l_dist=3)
                if match_item is None:
                    print("Failed to find a match to query", item)
                    continue
                else:
                    print("Best match: ", match_item)
                    item_ids = lvis[match_item]
            print("Found",len(item_ids),"objects for",item)
            if isinstance(index,int):
                if index >= len(item_ids):
                    raise ValueError("Index {} is out of range".format(index))
                item_ids = [item_ids[index]]
            elif index == 'random':
                import random
                item_ids = [random.choice(item_ids)]
            elif index == 'all':
                pass
            else:
                raise ValueError("Invalid index {}".format(index))
            ids.extend(item_ids)
    return objaverse.load_objects(ids)


def load(uids_or_queries : Union[str,List[str]],
         index : Union[str,int] = 'all',
         flattened = True,
         scale = 'auto',
         output_directory = None) -> Union[Geometry3D, List[Geometry3D]]:
    """Loads one or more objects from objaverse into a Geometry3D.
    
    Args:
        uids_or_queries: one or more queries or UIDs to load.
            E.g., ['alarm clock','ambulance','apple']
        index: the i'th index in the object list
        flattened: whether to flatten the object into a single mesh
        scale : 'auto' to determine the size from the object's bounding box,
            or a float to scale the object by that factor.
        output_directory: the directory to save the downloaded objects.
    """
    objects = download(uids_or_queries, index=index, flattened=flattened, scale=scale, output_directory=output_directory)
    if not objects:
        raise ValueError("No objects found for the given queries or UIDs.")

    geoms = []    
    for i, objfile in enumerate(objects.values()):
        s = trimesh.load(objfile)
        if s is None:
            raise ValueError("Failed to load object from file {}".format(objfile))
        s.convert_units('meters')
        res = from_trimesh(s, flatten=flattened)
        if isinstance(res, Geometry3D):
            geoms.append(res)
        else:
            # result is a WorldModel, convert to Group geometry
            geom = Geometry3D()
            geom.setGroup()
            for j,g in enumerate(res.rigidObjects):
                geom.setElement(j, g.geometry())
            geoms.append(geom)

        flipyz = so3.from_axis_angle(([1,0,0],math.pi/2))
        if scale == 1.0:
            T = (flipyz, [0, 0, 0])
        elif isinstance(scale, (int,float)):
            T = (flipyz, [0, 0, 0])
        elif scale == 'auto':
            bb = geom.getBBTight()
            print("Bounding box",bb)
            maxdim = max(b-a for (a,b) in zip(bb[0],bb[1]))
            print("Max dimension",maxdim)
            if maxdim > 250:
                print("Assuming object scale is in millimeters")
                _scale = MM_TO_METERS
            elif maxdim > 3:
                print("Assuming object scale is in inches")
                _scale = INCHES_TO_METERS
            else:
                _scale = 1.0
            #Need to flip the object around the x axis because y is up in objaverse
            T = (vectorops.mul(flipyz,_scale),[0,0,0])
        geoms[-1].transform(*T)
    if len(geoms) == 1:
        return geoms[0]
    else:
        return geoms
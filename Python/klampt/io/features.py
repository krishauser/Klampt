"""Module for working with feature mappings that map hierarchical (JSON-like)
objects to/from vectors (lists of extracted objects; typically primitive data
types).

Some terminology:
- A hierarchical object is a class or a potentially nested collection of dicts,
  lists (or tuples), and primitive objects (usually bool,int,float,str).
- A feature path in a hierarchical object is key or a list of keys of the
  collection.  These keys may only be strings (which can be attribute names or dict
  keys) and integers (which may be dict keys or list indices).
- A feature mapping is a list of feature paths, each of which maps to a node in
  the hierarchical object.
- A feature vector is a flattened list of extracted objects that is extracted
  from a hierarchical object, given a feature mapping.
- A feature vector can also be injected into a hierarchical object of the same
  structure, given the same feature mapping.

Compatible with numpy arrays, which are flattened into their constituent values.
"""

import sys
import numpy as np
from typing import Union, List, Dict, Tuple, Any

if sys.version_info[0] < 3:
    raise ImportError("Only works for Python 3.X")

def _extract_one(object, feature : Union[str,int,list,tuple]):
    if isinstance(feature,(list,tuple)):
        if len(feature) > 1:
            #nested feature
            return _extract_one(_extract_one(object,feature[0]),feature[1:])
        else:
            #base case of nesting
            feature = feature[0]
    if isinstance(feature,int):
        return object[feature]
    elif isinstance(feature,str) and hasattr(object,feature):
        return getattr(object,feature)
    else:
        return object[feature]

def _flatten(object) -> list:
    """Given a JSON-like object of lists, tuples, dicts, numpy arrays,
    or primitive values, flattens all of the values in object into a
    single list.
    """
    if isinstance(object,np.ndarray):
        return object.flatten().tolist()
    elif isinstance(object,(list,tuple)):
        return sum([_flatten(v) for v in object],[])
    elif isinstance(object,dict):
        return sum([_flatten(v) for v in object.values()],[])
    else:
        return [object]

def extract(object : Any, features: List[Union[str,int,list,tuple]]) -> list:
    """Given a hierarchical object 'object' and a list of feature paths
    'features', returns the values of object indexed by those feature paths
    as a flattened list.
    
    Note: a path in features may reference an internal node,
    in which case the return result will contain all values under that
    internal node.

    A simple example::

        object = {'name':'Joe','account':1234,'orders':[2345,3456]}
        features = ['account','orders']
        extract(object,features) # => [1234,2345,3456]

    A more complex example::

        features = ['account',['orders',0],['orders',1]]
        extract(object,features) # => [1234,2345,3456]

    Note: feature paths may only be strings, integers, or lists of strings
    and integers.
    """
    v = []
    for f in features:
        try:
            v.append(_extract_one(object,f))
        except Exception:
            print("Error extracting feature",f,"from",object)
            raise
    return _flatten(v)

def _fill(object,valueIter):
    if isinstance(object,np.ndarray):
        shape = object.shape
        object[:] = np.array([next(valueIter) for _ in range(np.prod(shape))]).reshape(shape)
    elif isinstance(object,(list,tuple)):
        for i in range(len(object)):
            if hasattr(object[i],'__iter__'):
                _fill(object[i],valueIter)
            else:
                object[i] = next(valueIter)
    elif isinstance(object,dict):
        for i in object:
            if hasattr(object[i],'__iter__'):
                _fill(object[i],valueIter)
            else:
                object[i] = next(valueIter)
    else:
        raise RuntimeError("_fill can only be called with a container type")

def _inject_one(object,feature,valueIter):
    if isinstance(feature,(list,tuple)):
        if len(feature) > 1:
            _inject_one(_extract_one(object,feature[0]),feature[1:],valueIter)
            return
        else:
            feature = feature[0]
    if isinstance(feature,int):
        if hasattr(object[feature],'__iter__'):
            _fill(object[feature],valueIter)
        else:
            object[feature]=next(valueIter)
    elif hasattr(object,feature):
        if hasattr(getattr(object,feature),'__iter__'):
            _fill(getattr(object,feature),valueIter)
        else:
            setattr(object,feature,next(valueIter))
    else:
        if hasattr(object[feature],'__iter__'):
            _fill(object[feature],valueIter)
        else:
            object[feature]=next(valueIter)

def inject(object : Any, features : List[Union[str,int,list,tuple]], values: List) -> None:
    """Given a hierarchical structure 'object', a list of feature paths
    'features', and a list of values 'values',
    sets those values of object indexed by those feature names
    to the corresponding entries in 'values'.
    
    Note: the feature paths may reference internal nodes, in which case the
    internal nodes are extracted

    A simple example::

        object = {'name':'Joe','account':1234,'orders':[2345,3456]}
        features = ['account','orders']
        inject(object,features,[1235,2346,3457]]) # => object now 
              #contains {'name':'Joe','account':1235,'orders':[2346,3457]}

    A more complex example::

        features = [['orders',1]]
        inject(object,features,[3458]) #=> object now contains
             #{'name':'Joe','account':1235,'orders':[2346,3458]}

    Note: features may only be strings, integers, or lists of strings
    and integers.
    """
    viter = iter(values)
    for f in features:
        _inject_one(object,f,viter)


def structure(object : Any, hashable=True) -> Any:
    """Returns an object describing the hierarchical structure of the given
    object (eliminating the values).  Structures can be then compared via
    equality testing.  This can be used to more quickly compare
    two structures than structureMatch, particularly when hashable=True.

    If hashable = True, this returns a hashable representation.  Otherwise,
    it returns a more human-readable representation.
    """
    if isinstance(object,np.ndarray):
        return object.shape
    elif isinstance(object,(list,tuple)):
        res= [structure(v) for v in object]
        if all(v is None for v in res):
            #return a raw number
            return len(res)
        if hashable: return tuple(res)
        return res
    elif isinstance(object,dict):
        res = dict()
        for k,v in object.items():
            res[k] = structure(v)
        if hashable: return tuple(res.items())
        return res
    else:
        return None

def structureMatch(object1 : Any, object2 : Any) -> bool:
    """Returns true if the objects have the same hierarchical structure
    (but not necessarily the same values)."""
    if isinstance(object1,np.ndarray):
        if not isinstance(object2,np.ndarray): return False
        return object1.shape == object2.shape
    elif isinstance(object1,(list,tuple)):
        if not isinstance(object2,(list,tuple)): return False
        if len(object1) != len(object2): return False
        for (a,b) in zip(object1,object2):
            if not structureMatch(a,b): return False
        return True
    elif isinstance(object1,dict):
        if not isinstance(object2,dict): return False
        if len(object1) != len(object2): return False
        for k,v in object1.items():
            try:
                v2 = object2[k]
            except KeyError:
                return False
            if not structureMatch(v,v2): return False
        return True
    if hasattr(object1,'__iter__'):
        if not hasattr(object2,'__iter__'):
            return False
        #TODO: check other collections?
        return True
    else:
        if hasattr(object2,'__iter__'):
            return False
        #TODO: check for compatibility between classes?
        return True

def schema(object : Any) -> Union[None,List,Dict]:
    """Returns an object describing the hierarchical structure of the given
    object, with Nones in place of the values.  During schemaMatch, None's
    match with any value.  The None values can also be replaced with values
    to enforce specific value matches, or boolean predicates to enforce
    more general matches.
    """
    if isinstance(object,np.ndarray):
        return np.full(object.shape, None).tolist()
    if isinstance(object,(list,tuple)):
        res= [schema(v) for v in object]
        return res
    elif isinstance(object,dict):
        res = dict()
        for k,v in object.items():
            res[k] = structure(v)
        return res
    else:
        return None

def schemaMatch(schema : Any, object : Any) -> bool:
    """Returns true if the object matches the given schema."""
    if schema is None: return True
    if isinstance(schema,(list,tuple)):
        if not isinstance(object,(list,tuple)): return False
        if len(schema) != len(schema): return False
        for (a,b) in zip(schema,object):
            if not schemaMatch(a,b): return False
        return True
    elif isinstance(schema,dict):
        if not isinstance(schema,dict): return False
        if len(schema) != len(object): return False
        for k,v in schema.items():
            try:
                v2 = object[k]
            except KeyError:
                return False
            if not schemaMatch(v,v2): return False
        return True
    else:
        return (schema==object)


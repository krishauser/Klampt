from __future__ import print_function
import sys
from collections import defaultdict

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

#all of the classes defined in the package that you'd like to cross-reference
classes = {'RobotModel','WorldModel','RobotModelLink','Terrain','RigidObjectModel',
    'Geometry3D','Appearance','TriangleMesh','PointCloud','VolumeGrid','GeometricPrimitive',
    'ContactParameters','Mass','DistanceQueryResult','DistanceQuerySettings',
    'SimRobotSensor','SimRobotController','SimBody','Simulator',
    'CSpaceInterface','MotionPlanInterface',
    'ContactPoint','Trajectory','RobotTrajectory','HermiteTrajectory','MultiPath',
    'CSpace','MotionPlan'
    }
#default prefix for class cross-references
default_class_prefix = 'klampt'
#other prefixes for class cross-references
class_prefix = {
    'ContactPoint':'klampt.model.contact',
    'Trajectory':'klampt.model.trajectory',
    'RobotTrajectory':'klampt.model.trajectory',
    'HermiteTrajectory':'klampt.model.trajectory',
    'MultiPath':'klampt.model.multipath',
    'CSpace':'klampt.plan.cspace',
    'MotionPlan':'klampt.plan.cspace',
}

basic_types = {'int','float','str','bytes','bool'}

#conversions from SWIG docstrings to RST docstrings
to_python_types = { 'char const *':'str',
    'char *':'str',
    'char':'str',
    'std::string':'str',
    'double':'float',
    'doubleVector': "list of floats",
    'doubleArray': "list of floats",
    'doubleMatrix': "2D matrix, list of list of floats",
    'floatVector': "list of floats",
    'floatArray': "list of floats",
    'intVector': "list of int",
    'intArray': "list of int",
    'stringArray': "list of str",
    'double const [3]': "list of 3 floats",
    'double [3]': "list of 3 floats",
    'double const [9]': "list of 9 floats (so3 element)",
    'double [9]': "list of 9 floats (so3 element)",
    'std::vector< unsigned char,std::allocator< unsigned char > > const':'bytes',
    'PyObject': "object"
}

to_python_defaults = {'NULL':"None"}

def parse_type(typestr):
    typestr = typestr.strip()
    try:
        return to_python_types[typestr]
    except KeyError:
        pass
    processed = False
    if typestr.endswith('&') or typestr.endswith('*'):
        typestr = typestr[:-1].strip()
        processed = True
    if typestr.endswith('const'):
        typestr = typestr[:-5].strip()
        processed = True
    if typestr.endswith('const '):
        typestr = typestr[:-6].strip()
        processed = True
    if processed:
        try:
            return to_python_types[typestr]
        except KeyError:
            pass
    return typestr

def parse_default(defstr):
    return to_python_defaults.get(defstr,defstr)

def to_type_doc(type):
    if type in classes:
        return ':class:`~%s.%s`'%(class_prefix.get(type,default_class_prefix),type)
    elif type in basic_types:
        return type
    else:
        return ':obj:`%s`'%(type,)

def print_signature(siglist,indent0,docstring):
    """Converts a SWIG signature to Google Python documentation convention.
    """
    docsummary = []
    docdetails = []
    print_args = True
    print_return = True
    reading_summary = True
    for ln in docstring:
        sln = ln.strip()
        if len(sln) > 0:
            if reading_summary:
                docsummary.append(ln)
            else:
                docdetails.append(ln)
            if sln.startswith('Args:'):
                print_args = False
            if sln.startswith('Returns:'):
                print_return = False
        else:
            #read whitespace
            if reading_summary:
                if len(docsummary) > 0:
                    reading_summary = False
                else:
                    #ignore
                    pass
            else:
                docdetails.append(ln)

    for ln in docsummary:
        print(ln)
    if len(docsummary) > 0:
        print()

    indentstr = " "*(indent-1)
    fn = None
    args = []
    ret = []
    for s in siglist:
        args.append([])
        parts = s.split(" -> ")
        assert len(parts) in [1,2]
        #parse arguments
        s = parts[0]
        fn = s[:s.find("(")]
        sargs = s[s.find("(")+1:s.find(")")]
        if len(sargs.strip()) > 0:
            for arg in sargs.split(','):
                try:
                    atype,aname = arg.rsplit(' ',1)
                except Exception:
                    eprint("Couldnt parse argument",arg,"?")
                    raise
                if aname == 'self':
                    #skip documenting self
                    continue
                aparts = aname.split('=',2)
                if len(aparts)>1:
                    eprint("Parts",aparts[0],aparts[1])
                    args[-1].append((aparts[0],parse_type(atype),parse_default(aparts[1])))
                else:
                    args[-1].append((aname,parse_type(atype),None))

        #parse return value
        if len(parts) == 2:
            ret.append(parse_type(parts[1]))
        else:
            ret.append('None')
    if len(siglist) == 1:
        if len(args[0]) > 0 and print_args:
            print(indent0+"Args:")
            for (aname,atype,adef) in args[0]:
                if adef is None:
                    print(indent0+indentstr,aname,'(%s)'%(to_type_doc(atype),))
                else:
                    print(indent0+indentstr,aname,'(%s, optional): default value %s'%(to_type_doc(atype),adef))
        if ret[0] is not 'None' and fn != '__init__'and print_return:
            print(indent0+"Returns:")
            print(indent0+indentstr,"(%s):"%(to_type_doc(ret[0])))
    else:
        #try determining argument types and optional
        aorders = defaultdict()
        acounts = defaultdict(int)
        atypes = defaultdict(list)
        adefaults = defaultdict(list)
        for asig in args:
            for i,(aname,atype,adef) in enumerate(asig):
                if aname not in aorders:
                    aorders[aname] = len(acounts)
                acounts[aname] = acounts[aname] + 1
                atypes[aname].append(atype)
                adefaults[aname].append(adef)
        #output the function signatures
        fndocstrings = []
        for asig,aret in zip(args,ret):
            argstr = '()'
            retstr = ''
            if len(asig) > 0:
                argstr = '(%s)'%(','.join((aname if adef is None else aname+'='+adef) for (aname,atype,adef) in asig))
            if aret != 'None':
                retstr = ': '+to_type_doc(aret)
            if argstr+retstr not in fndocstrings:
                print(indent0+fn,argstr+retstr)
                print()
                fndocstrings.append(argstr+retstr)
        if print_args:
            print()
            #output the Args: section
            print(indent0+'Args:')
            alist = [(n,o) for (n,o) in aorders.iteritems()]
            alist = sorted(alist,key=lambda item:item[1])
            for (arg,i) in alist:
                typestr = ''
                if len(atypes[arg]) == 1:
                    typestr = to_type_doc(atypes[arg][0])
                else:
                    typeset = set(atypes[arg])
                    typestr = ' or '.join([to_type_doc(a) for a in typeset])
                if acounts[arg] < len(args):
                    eprint("Argument",arg,"to",fn,"appears optional?",acounts[arg],len(args))
                    typestr = typestr + ', optional'
                desc = ''
                defset = list(set(v for v in adefaults[arg] if v is not None))
                if len(defset) == 1:
                    desc = 'default value '+defset[0]
                elif len(defset) > 1:
                    desc = 'default values: '+','.join(defset)
                print(indent0+indentstr,arg,'(%s): %s'%(typestr,desc))
        if any(r != 'None' for r in ret) and fn != '__init__' and print_return:
            print()
            print(indent0+'Returns:')
            unique = set(ret)
            print(indent0+indentstr,'(%s):'%(' or '.join([to_type_doc(r) for r in unique]),))

    if len(docdetails) > 0:
        print()
    for ln in docdetails:
        print(ln)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python improve_swig_docs.py in_py_file > out_py_file")
        exit(0)

    f = open(sys.argv[1],'r')
    reading_docstring = False
    finished_reading_signature = False
    current_signature = []
    current_docstring = []
    outer_indent = ''

    indent = 4
    for line,ln in enumerate(f.readlines()):
        ln = ln[:-1]  #strip endline
        first_nw = len(ln) - len(ln.lstrip())
        quote = (ln[first_nw:first_nw+3]=='"""')
        if reading_docstring:
            if quote:
                #end parsing
                eprint("ENDING PARSING line",line)
                reading_docstring = False
                #empty line, could be the start or the end of the function signature
                if len(current_signature) > 0:
                    print_signature(current_signature,outer_indent,current_docstring)
                else:
                    for priorline in current_docstring:
                        print(priorline)
                print(ln)
            elif finished_reading_signature:
                #add to docstring
                current_docstring.append(ln)
            else:
                #determine whether to process
                if ln.find('(') >= 0 and ln.find(')') >= 0:
                    fn_sig = ln.strip()
                    eprint("Parsing signature for",fn_sig)
                    current_signature.append(fn_sig)
                else:
                    finished_reading_signature = True
                    current_docstring.append(ln)
        elif quote:
            if ln.count('"""') == 1:
                eprint("STARTING PARSING line",line)

                reading_docstring = True
                current_signature = []
                current_docstring = []
                finished_reading_signature = False
                outer_indent = ln[:first_nw]
            print(ln)
        else:
            print(ln)

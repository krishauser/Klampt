
import sys
from collections import defaultdict
from improve_swig_docs_settings import *
import re

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def parse_type(typestr,argname=None):
    typestr = typestr.strip()
    try:
        return to_python_types[typestr]
    except KeyError:
        pass
    if argname is not None:
        try:
            return to_python_types[typestr+' '+argname]
        except KeyError:
            pass
    processed = False
    if typestr.endswith('&'):
        typestr = typestr[:-1].strip()
        processed = True
    if 'const' in typestr:
        typestr = typestr.replace(' const','')
        processed = True
    if processed:
        try:
            return to_python_types[typestr]
        except KeyError:
            pass
    return typestr

def parse_type_hint(typestr):
    typestr0 = typestr
    typestr = typestr.strip().strip('"')
    try:
        return to_python_type_hints[typestr]
    except KeyError:
        try:
            return to_python_types[typestr]
        except KeyError:
            pass
    processed = False
    if typestr.endswith('&'):
        typestr = typestr[:-1].strip()
        processed = True
    if 'const' in typestr:
        typestr = typestr.replace(' const','')
        processed = True
    if '::' in typestr:  #SWIG versions of STL containers have lots of boilerplate
        for k,v in stl_to_python_type_hints.items():
            if k in typestr:
                return v
    if processed:
        try:
            return to_python_type_hints[typestr]
        except KeyError:
            try:
                return to_python_types[typestr]
            except KeyError:
                pass
    return typestr0

def parse_default(defstr):
    return to_python_defaults.get(defstr,defstr)

def to_type_doc(type):
    if type in classes:
        return ':class:`~%s.%s`'%(class_prefix.get(type,default_class_prefix),type)
    elif type in basic_types:
        return type
    else:
        return ':obj:`%s`'%(type,)

def smart_split(string : str, delim=',', quotes='"'):
    """Splits a string by a delimiter, ignoring parts within quotes.

    Quotes can also be a 2-element list, which indicates start and end tokens.
    """
    if len(quotes)==2 and isinstance(quotes,(list,tuple)):
        start,end=quotes
        quote_parts = []
        i = 0
        last = 0
        in_quote = False
        num_quotes = 0
        ever_quoted = False
        while i < len(string):
            if string[i:].startswith(start):
                ever_quoted = True
                if in_quote:
                    eprint("Adding quote at position",i)
                else:
                    quote_parts.append(string[last:i])
                    last = i
                num_quotes += 1
                in_quote = True
                i += len(start)
            elif string[i:].startswith(end):
                if not in_quote:
                    assert num_quotes == 0
                    raise RuntimeError("End terminator '{}' encountered at position {} outside of quote: {}".format(end,i,string))
                num_quotes -= 1
                eprint("Removing quote at position",i)
                i += len(end)
                if num_quotes == 0:
                    in_quote = False
                    eprint("Ended quoted region",string[last:i])
                    quote_parts.append(string[last:i])
                    last = i
            else:
                i += 1
        if last != len(string):
            quote_parts.append(string[last:])
        # if ever_quoted:
        #     eprint("Split string",string,"->",quote_parts)
        #     input()
    else:
        quote_parts = string.split(quotes)
    split_args = []
    for i,part in enumerate(quote_parts):
        if i%2==0:
            split_parts = part.split(delim)
            if len(split_parts) > 0:
                if split_parts[0]=='':
                    split_args += split_parts[1:]
                elif len(split_args) > 0:
                    split_args[-1] = split_args[-1]+split_parts[0]
                    split_args += split_parts[1:]
                else:
                    split_args += split_parts
        else:
            split_args[-1]+='"{}"'.format(part)
    return split_args


def print_definition(defn : str, indent0 : str):
    fn,args,ret = re.split('\(|\)',defn)
    #Shouldn't do normal args.split(',') since nested vector and map type hints can have commas in them
    args = smart_split(args,',','"')
    # if '=' in args:
    #     eprint(defn,":",args,"->",split_args)
    #     input("HAS DEFAULT")

    #parse typing of arguments
    for i,arg in enumerate(args):
        if ':' not in arg: continue
        name,desc = arg.split(':',1)
        if '=' in desc: #has default arg
            desc,default_arg = desc.split('=')
            desc_parsed = parse_type_hint(desc)
            if desc_parsed == desc:
                if desc.strip().strip('"') not in classes:
                    eprint("Failed to parse arg type",desc)
                    #input()
            if desc_parsed is not None:
                # if ' ' in desc_parsed.strip():
                #     eprint("Uh... didn't catch RST type?",desc,desc_parsed)
                #     input()
                desc_parsed = desc_parsed + '=' + default_arg
            else:
                desc_parsed = '=' + default_arg
            # eprint("PARSED VERSION OF ARG WITH DEFAULT:",desc_parsed)
            # input()
        else:
            desc_parsed = parse_type_hint(desc)
            if desc_parsed == desc:
                if desc.strip().strip('"') not in classes:
                    eprint("Failed to parse arg type",desc)
                    #input()
        if desc_parsed is not None:
            # if ' ' in desc_parsed.strip():
            #     eprint("Uh... didn't catch RST type?",desc,desc_parsed)
            #     input()
            args[i] = name + ': ' + desc_parsed
        else:
            args[i] = name

    #parse typing of return value
    if '->' in ret:
        prefix,retval = ret.split('->',1)
        if retval.endswith(':'):
            suffix = ':'
            retval = retval[:-1]
        else:
            input("Uh... function definition doesn't end with :?")
        
        retval_parsed = parse_type_hint(retval)
        if retval_parsed == retval:
            if retval.strip().strip('"') not in classes:
                eprint("Couldn't parse return type",retval)
                #input()
        
        if retval_parsed is not None:
            ret = prefix+'->'+retval_parsed+suffix
        else:
            ret = prefix + suffix
    print("{}{}({}){}".format(indent0,fn,','.join(args),ret))

def print_signature(siglist,indent0,docstring):
    """Converts a SWIG signature in a docstring to Google
    Python documentation convention.

    Args:
        siglist (list of str): lines of SWIG signature in docstring
        indent0 (str): spaces giving the top-level indent
        docstring (list of str): remaining lines of docstring
    
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
            for arg in smart_split(sargs,',',['<','>']):
                if arg=='self':
                    continue
                try:
                    atype,aname = arg.rsplit(' ',1)
                except Exception:
                    eprint("Couldnt parse argument '{}' ?".format(arg))
                    eprint(sargs)
                    raise
                if aname == 'self':
                    #skip documenting self
                    continue
                aparts = aname.split('=',2)
                if len(aparts)>1:
                    eprint("Parts",aparts[0],aparts[1])
                    args[-1].append((aparts[0],parse_type(atype,aparts[0]),parse_default(aparts[1])))
                else:
                    args[-1].append((aname,parse_type(atype,aname),None))

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
            pass  #The type hints will already document the type
            #print(indent0+"Returns:")
            #print(indent0+indentstr,"%s:"%(to_type_doc(ret[0])))
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
            alist = [(n,o) for (n,o) in aorders.items()]
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
            if len(unique) > 1:
                print(indent0+indentstr,'(%s):'%(' or '.join([to_type_doc(r) for r in unique]),))
            else:
                print(indent0+indentstr,'%s:'%(''.join([to_type_doc(r) for r in unique]),))

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
    typing_injected = False
    current_signature = []
    current_docstring = []
    outer_indent = ''

    indent = 4
    for line,ln in enumerate(f.readlines()):
        ln = ln[:-1]  #strip endline
        first_nw = len(ln) - len(ln.lstrip())
        defn = ln[first_nw:first_nw+4]=='def '
        quote = (ln[first_nw:first_nw+3]=='"""' or ln[first_nw:first_nw+4]=='r"""')
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
                    if fn_sig[0] == '%':
                        finished_reading_signature = True
                        pos = ln.find('%')
                        ln = ln[:pos]+ln[pos+1:]
                        current_docstring.append(ln)
                    else:
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
        elif defn:
            print_definition(ln[first_nw:],ln[:first_nw])
        elif not typing_injected and typing_inject_location in ln:
            print(ln)
            print(typing_header)
            typing_injected = True
        else:
            print(ln)

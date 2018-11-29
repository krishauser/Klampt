from symbolic import *
from symbolic import _infix_operators,_prefix_operators,_builtin_functions
from ..io import loader
import json
from json import encoder
import weakref

VAR_PREFIX = ''
USER_DATA_PREFIX = '$'
NAMED_EXPRESSION_TAG = '#'
NAMED_EXPRESSION_PREFIX = '@'

_operator_precedence = {'pow':1,
    'mul':2,'div':2.5,
    'add':3,'sum':3,'sub':3.5,
    'neg':4,
    'not':5,
    'and':6,'or':6,
    'ge':7,'le':7,'eq':7,'ne':7}

#just a helper class to do some duck-typing
class _Object(object):
    pass

def byteify(input):
    """Helpful for converting unicode values in JSON loaded objects to strings"""
    if isinstance(input, dict):
        return {byteify(key): byteify(value)
                for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [byteify(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

class _TaggedExpression(Expression):
    def __init__(self,name):
        self.name = name
        Expression.__init__(self)

def indent(s,spaces):
    if spaces <= 0: return s
    return s.replace('\n','\n'+' '*spaces)

def _prettyPrintExpr(expr,astr,parseCompatible):
    """Returns a string representing this expression, where astr is a list of strings
    representing each argument"""
    if not isinstance(expr,OperatorExpression):
        return exprToStr(expr,parseCompatible)
    if len(expr.functionInfo.printers) > 0:
        if parseCompatible and 'parse' in expr.functionInfo.printers:
            return expr.functionInfo.printers['parse'](expr,astr)
        if not parseCompatible and 'str' in expr.functionInfo.printers:
            return expr.functionInfo.printers['str'](expr,astr)
    if expr.functionInfo.name in _prefix_operators:
        prefix = _prefix_operators[expr.functionInfo.name]
        assert len(expr.args) == 1,"Weird, prefix operator %s has %d arguments? %s"%(expr.functionInfo.name,len(astr),",".join(astr))
        return prefix + astr[0]
    if expr.functionInfo.name in _infix_operators:
        assert len(expr.args) == 2,"Weird, infix operator %s has %d arguments? %s"%(expr.functionInfo.name,len(astr),",".join(astr))
        return astr[0] + _infix_operators[expr.functionInfo.name] + astr[1]
    if expr.functionInfo.name == 'setitem':
        vconst = to_const(expr.args[0])
        iconst = to_const(expr.args[1])
        if vconst is not None and iconst is not None:
            if hasattr(iconst,'__iter__'):
                indexset = set(iconst)
                if parseCompatible:
                    astr[0] = '[' + ','.join(['0' if i in indexset else str(v) for i,v in enumerate(vconst)])+']'
                else:
                    astr[0] = '[' + ','.join(['*' if i in indexset else str(v) for i,v in enumerate(vconst)])+']'
    if expr.functionInfo.name == 'getitem':
        if isinstance(expr.args[0],OperatorExpression) and astr[0][0] != '(' and expr.args[0].functionInfo.name in _infix_operators:
            astr[0] = '(' + astr[0] + ')'
        #if expr.functionInfo.name == 'getslice':
        #    if len(astr) <= 2:
        #        astr.append('')
        #    if len(astr) <= 3:
        #        astr.append('')
        #    return astr[0] + '[%s:%s:%s]'%(astr[1],astr[2],astr[3])
        if isinstance(expr.args[1],slice):
            start,stop,step = expr.args[1].start,expr.args[1].stop,expr.args[1].step
            if expr.args[1].stop > 90000000000:
                astr[1] = str(start)+":"
            else:
                astr[1] = str(start)+":"+str(expr.args[1].stop)
            if step is not None:
                astr[1] = astr[1] + ":" + str(step)
        return astr[0] + '[' +astr[1] + ']'
    #default
    if len(astr) > 1 and sum(len(a) for a in astr) > 80-2-len(expr.functionInfo.name):
        res = expr.functionInfo.name + "("
        res += ',\n  '.join([indent(a,2) for a in astr]) + ')'
    else:
        res = expr.functionInfo.name + "("
        res += ','.join(astr) + ')'
    return res

def _make_tagged(expr,prefix="SubExp"):
    """Creates a copy of expr where each reference to a common subexpression is
    replaced with a TaggedExpression.  If there are no common subexpressions,
    expr is returned."""
    def _refspre(node):
        if 'refs' in node._cache:
            node._cache['refs'] += 1
            return (False,True,node._cache['refs'])
        node._cache['refs'] = 1
        return (True,True,None)
    expr._traverse(pre=_refspre,cache=False)
    #all the cache values are now the number of references to a subexpression

    def _hassubexpr_pre(node):
        if node._cache['refs'] > 1:
            #print "Node",node.functionInfo.name,"is a repeated subexpression"
            node._cache['hassubexpr'] = True
            return (False,True,True)
        return (True,True,None)
    def _hassubexpr_post(node,cvals):
        if len(cvals) == 0:
            return (True,False)
        res = any(cvals)
        #print "Child of",node.functionInfo.name,"has repeated subexpression"
        node._cache['hassubexpr'] = res
        if res: return (True,True)
        return (True,False)
    if not expr._traverse(pre=_hassubexpr_pre,post=_hassubexpr_post,cache=False):
        #print "***Expression has no subexpressions***"
        expr._clearCache('refs')
        expr._clearCache('hassubexpr')
        return expr

    assert expr._cache.get('hassubexpr',False) == True
    expr._clearCache('refs')
    #print "***Expression has subexpressions***"
    subexprs = dict()
    def replace(node):
        if not node._cache.get('hassubexpr',False): return node
        if 'refs' in node._cache:
            if 'id' not in node._cache:
                #new detected subexpression, not added yet
                tag = prefix+str(len(subexprs)+1)
                node._cache['id'] = tag
                subexprs[tag] = _TaggedExpression(tag)
            node._cache['refs'] += 1
            #print "Reference",node._cache['refs'],"to",node.functionInfo.name
            return subexprs[node._cache['id']]
        node._cache['refs'] = 1
        if node._children is None:
            return node
        else:
            assert isinstance(node,OperatorExpression)
            #print "New reference to",node.functionInfo.name
            creps = [replace(c) for c in node._children]
            if any(cr is not c for (cr,c) in zip(creps,node._children)):
                return OperatorExpression(node.functionInfo,creps,node.op)
            else:
                return node
    repl = replace(expr)
    expr._clearCache('refs')
    expr._clearCache('hassubexpr')
    #NEED TO CLEAR 'id' from cache after repl is used
    return repl

def _to_jsonobj(val):
    if isinstance(val,(bool,int,float)):
        return val
    elif isinstance(val,(np.ndarray,np.float64)):
        return val.tolist()
    elif isinstance(val,(list,tuple)):
        return [_to_jsonobj(x) for x in val]
    else:
        try:
            return loader.toJson(val)
        except:
            raise ValueError("Unable to convert object "+repr(val)+" to JSON object")
    return None

def _json_complex(jsonval):
    if isinstance(jsonval,dict):
        return (len(jsonval) > 0)
    elif isinstance(jsonval,(list,tuple)):
        return any(_json_complex(v) for v in jsonval)
    else:
        return False

def _json_depth(jsonval):
    if isinstance(jsonval,dict):
        return 1 + max(_json_depth(v) for v in jsonval.itervalues())
    elif isinstance(jsonval,(list,tuple)):
        return 1 + max(_json_depth(v) for v in jsonval)
    else:
        return 1

def exprToStr(expr,parseCompatible=True,expandSubexprs='auto'):
    """Converts an Expression to a printable or parseable string.

    Arguments
    - expr: the Expression to convert
    - parseCompatible: if True, the result is readable via exprFromStr()
    - expandSubexprs: whether to expand subexpressions. Can be:
      'auto': if parseCompatible, equivalent to False. 
              if parseCompatible=False, equivalent to True.
      True: expands all common subexpressions
      False: does not expand common subexpressions.
      'show': Internally used.
    """
    if isinstance(expr,ConstantExpression):
        if isinstance(expr.value,slice):
            start,stop,step = expr.value.start,expr.value.stop,expr.value.step
            return "%s:%s%s"%(str(start),"" if stop > 900000000000 else str(stop),"" if step is None else ":"+str(step))
        try:
            jsonval = _to_jsonobj(expr.value)
        except:
            return str(expr.value)
        if parseCompatible:
            return json.dumps(jsonval)
        else:
            original_float_repr = encoder.FLOAT_REPR
            encoder.FLOAT_REPR = lambda o:format(o,'.14g')
            if _json_complex(jsonval):
                res = json.dumps(jsonval,sort_keys=True, indent=4, separators=(',', ': '))
            else:
                res = json.dumps(jsonval,sort_keys=True)
            encoder.FLOAT_REPR = original_float_repr
            return res
    elif isinstance(expr,VariableExpression):
        if parseCompatible:
            return VAR_PREFIX+expr.var.name
        else:
            return str(expr.var)
    elif isinstance(expr,UserDataExpression):
        return USER_DATA_PREFIX+expr.name
    elif isinstance(expr,OperatorExpression):
        if expandSubexprs == 'auto':
            expandSubexprs = not parseCompatible
        if expandSubexprs:
            astr = []
            for i,a in enumerate(expr.args):
                a._parent = (weakref.ref(expr),i)
                astr.append(exprToStr(a,parseCompatible,expandSubexprs))
                if not isinstance(a,OperatorExpression) and  expandSubexprs == 'show' and ('id' in a._cache or 'name' in a._cache):
                    #tagged subexprs need parenthesies
                    if astr[-1][-1] != ')':
                        astr[-1] = '('+astr[-1]+')'
                    astr[-1] = astr[-1] + NAMED_EXPRESSION_TAG + a._cache.get('id',a._cache.get('name'))
                a._parent = None
            res = _prettyPrintExpr(expr,astr,parseCompatible)
            if expandSubexprs == 'show' and ('id' in expr._cache or 'name' in expr._cache):
                #tagged subexprs need parenthesies
                if res[-1] != ')':
                    res = '('+res+')'
                return res + NAMED_EXPRESSION_TAG + expr._cache.get('id',expr._cache.get('name'))
            oldparent = expr._parent
            iscomplex = expr.depth() >= 0 and (expr.functionInfo.name in _operator_precedence)
            expr._parent = oldparent
            if iscomplex and (expr._parent is not None and not isinstance(expr._parent,str)):
                if parseCompatible:
                    return '(' + res + ')'
                else:
                    parent = expr._parent[0]()
                    if parent.functionInfo.name in _operator_precedence:
                        expr_precedence = _operator_precedence[expr.functionInfo.name]
                        parent_precedence = _operator_precedence[parent.functionInfo.name]
                        #if - is the first in a summation, don't parenthesize it
                        if expr._parent[1] == 0 and expr.functionInfo.name == 'neg' and parent.functionInfo.name in ['sum','add','sub']:
                            return res
                        if expr_precedence > parent_precedence:
                            return '(' + res + ')'
                        if expr_precedence == parent_precedence:
                            if expr.functionInfo is parent.functionInfo and expr.functionInfo.properties.get('associative',False):
                                return res
                            else:
                                return '(' + res + ')'
            return res
        else:
            if not parseCompatible:
                taggedexpr = _make_tagged(expr,"")
            else:
                taggedexpr = _make_tagged(expr)
            res = exprToStr(taggedexpr,parseCompatible,'show')
            if taggedexpr is not expr:
                expr._clearCache('id',deep=True)
            return res
    elif isinstance(expr,_TaggedExpression):
        return NAMED_EXPRESSION_PREFIX+expr.name
    elif is_const(expr):
        return str(expr)
    else:
        raise ValueError("Unknown type "+expr.__class__.__name__)


def exprFromStr(context,string,fmt=None,add=False):
    """Returns an Expression from a string.  In auto mode, this reads in constants in klampt.loader JSON-
    compatible format, standard variables in the form "x", user data in the form of strings prepended with $
    (e.g., "$x"), and named expression references in the form of strings prepended with @.

    - string: the string to parse.
    - fmt: specifies a format for the string.  Can be None (auto), 'auto', or 'json'
    - add: if true, adds all variables referenced in the string to the context.  Otherwise, undefined
      variables are referred to as user data.

    (Parsing is a little slow, so try not to use it in tight inner loops)
    """
    if len(string) == 0:
        raise ValueError("Empty string provided")
    if fmt == None:
        if string[0] == '{':
            fmt = 'json'
        else:
            fmt = 'auto'
    if fmt == 'auto':
        import re,ast
        USERDATA_MARKER = '___'
        EXPR_MARKER = '____'
        TAGLIST_NAME = '__tagexprlist__'
        taglist = context.expressions.copy()
        def __settag__(self,tagname,taglist):
            assert isinstance(tagname,ConstantExpression) and isinstance(tagname.value,str)
            taglist[tagname.value] = self
            return self
        def __gettag__(tagname,taglist):
            assert isinstance(tagname,ConstantExpression) and isinstance(tagname.value,str)
            return taglist[tagname.value]
        Expression.__settag__ = __settag__

        x = re.sub(r"\$(\w+)", r"___\1",string)
        x = re.sub(r"\#(\w+)", r'.__settag__("\1",__tagexprlist__)',x)
        x = re.sub(r"\@(\w+)", r'__gettag__("\1",__tagexprlist__)',x)
        #print "Substituted string",x
        tree = ast.parse(x,mode='eval')
        missing_functions = []
        missing_names = []
        userdata = {}
        #hack to easily access functions with the class.attribute syntax
        allFunctions = _builtin_functions.copy()
        for name,func in context.customFunctions.iteritems():
            path = name.split('.')
            if len(path) == 1:
                allFunctions[name] = func
            else:
                if path[0] not in allFunctions:
                    allFunctions[path[0]] = _Object() 
                root = allFunctions[path[0]]
                for n in path[1:-1]:
                    if not hasattr(root,n):
                        setattr(root,n,_Object())
                    root = getattr(root,n)
                setattr(root,path[-1],func)
        allFunctions[TAGLIST_NAME] = taglist
        allFunctions['__gettag__'] = __gettag__

        class RewriteVarNames(ast.NodeTransformer):
            def __init__(self):
                self.infunc = False
            def visit_Call(self,node):
                self.infunc = True
                self.generic_visit(node)
                return node
            def visit_Name(self, node):
                if self.infunc:
                    self.infunc = False
                    if node.id not in allFunctions:
                        missing_functions.append(node.id)
                    return node
                if node.id.startswith(USERDATA_MARKER):
                    basename = node.id[len(USERDATA_MARKER):]
                    userdata[node.id] = expr(basename)
                else:
                    if node.id in context.variableDict:
                        userdata[node.id] = expr(context.variableDict[node.id])
                    elif add:
                        userdata[node.id] = expr(context.addVar(node.id,'N'))
                    elif node.id == TAGLIST_NAME:
                        pass
                    else:
                        missing_names.append(node.id)
                        userdata[node.id] = expr(node.id)
                return node
            def visit_Num(self, node):
                return ast.copy_location(ast.Call(func=ast.copy_location(ast.Name(id="_const",ctx=ast.Load()),node),args=[node],keywords=[]),node)
            def visit_Str(self, node):
                return ast.copy_location(ast.Call(func=ast.copy_location(ast.Name(id="_const",ctx=ast.Load()),node),args=[node],keywords=[]),node)
            def visit_List(self, node):
                args = []
                for idx, item in enumerate(node.elts):
                    args.append(self.visit(item))
                return ast.copy_location(ast.Call(func=ast.copy_location(ast.Name(id="_convert_list",ctx=ast.Load()),node),args=args,keywords=[]),node)
            def visit_Tuple(self, node):
                args = []
                for idx, item in enumerate(node.elts):
                    args.append(self.visit(item))
                return ast.copy_location(ast.Call(func=ast.copy_location(ast.Name(id="_convert_list",ctx=ast.Load()),node),args=args,keywords=[]),node)

        #print "old tree:",ast.dump(tree)
        newtree = RewriteVarNames().visit(tree)
        #print "new tree:",ast.dump(newtree)
        if len(missing_functions) > 0:
            raise ValueError("Undefined functions "+','.join(missing_functions))
        if len(missing_names) > 0:
            raise ValueError("Undefined variable "+','.join(missing_names))
        allFunctions['_const'] = const
        allFunctions['_convert_list'] = lambda *args:array(*args)
        ctree = compile(newtree, filename="<ast>", mode="eval")
        res = eval(ctree,allFunctions,userdata)
        delattr(Expression,'__settag__')
        return res
    elif fmt == 'json':
        import json
        obj = json.parse(str)
        return exprFromJson(context,obj)
    else:
        raise ValueError("Invalid format "+fmt)

def exprToJson(expr):
    if isinstance(expr,ConstantExpression):
        return _to_jsonobj(expr.value)
    elif isinstance(expr,UserDataExpression):
        return USER_DATA_PREFIX+expr.name
    elif isinstance(expr,VariableExpression):
        return VAR_PREFIX+expr.var.name
    elif isinstance(expr,OperatorExpression):
        def _tojson(node,childvals):
            if isinstance(node,OperatorExpression):
                res = {"type":expr.functionInfo.name}
                res["args"] = childvals
                if 'id' in node._cache:
                    res['id'] = node._cache['id']
                return True,res
            else:
                return True,exprToJson(node)
        taggedexpr = _make_tagged(expr)
        res = taggedexpr._traverse(post=_tojson,cacheas='json')
        if taggedexpr is not expr:
            expr._clearCache('id',deep=True)
        return res
    elif isinstance(expr,_TaggedExpression):
        return NAMED_EXPRESSION_PREFIX+expr.name


def exprFromJson(context,jsonObj,taggedExpressions=None):
    """Creates an Expression from a JSON object previously saved by expr.toJson()"""
    #print "exprFromJson:",jsonObj
    name = str(jsonObj['type'])
    args = jsonObj['args']
    parsedArgs = []
    if taggedExpressions is None:
        taggedExpressions = dict()
    for a in args:
        if isinstance(a,str):
            if a.startswith(USER_DATA_PREFIX):
                #user data reference
                plen = len(USER_DATA_PREFIX)
                parsedArgs.append(context.userData[a[plen:]])
            elif a.startswith(NAMED_EXPRESSION_PREFIX):
                plen = len(NAMED_EXPRESSION_PREFIX)
                a = a[plen:]
                if a in taggedExpressions:
                    parsedArgs.append(taggedExpressions[a])
                elif a in context.expressions:
                    parsedArgs.append(context.expressions[a])
                else:
                    print "exprFromJson(): Valid tags:",taggedExpressions.keys(),"(tags)",context.expressions.keys(),"(expressions)"
                    raise RuntimeError("Invalid expression tag "+NAMED_EXPRESSION_PREFIX+a)
            else:
                #variable reference
                if a not in context.variableDict:
                    raise RuntimeError("Invalid variable reference "+a)
                parsedArgs.append(context.variableDict[a])
        elif isinstance(a,dict):
            #assume it's an expression or a loader object
            try: 
                parsedArgs.append(exprFromJson(context,a,taggedExpressions))
            except KeyError:
                try: 
                    parsedArgs.append(loader.fromJson(a))
                except Exception:
                    raise ValueError("Error parsing JSON object %s into expression or loader object"%(str(a),))
            if 'id' in a:
                assert a['id'] not in taggedExpressions,"Multiply defined tag "+str(a['id'])
                taggedExpressions[a['id']] = parsedArgs[-1]
        else:
            parsedArgs.append(a)
    if name in _builtin_functions:
        return _builtin_functions[name](*parsedArgs)
    if name in context.customFunctions:
        return context.customFunctions[name](*parsedArgs)
    raise RuntimeError("Invalid expression type "+name)

def typeToJson(type):
    res = { 'char':type.char }
    if type.size is not None:
        res['size'] = type.size
    if type.subtype is not None:
        if isinstance(type.subtype,str):
            res['subtype'] = type.subtype
        elif isinstance(type.subtype,list):
            res['subtype'] = [typeToJson(st) for st in type.subtype]
        else:
            res[subtype] = typeToJson(type.subtype)
    return res

def typeFromJson(jsonObj):
    assert 'char' in jsonObj
    st = None
    if 'subtype' in jsonObj:
        subtypeobj = jsonObj['subtype']
        if isinstance(subtypeobj,list):
            st = [typeFromJson(stobj) for stobj in subtypeobj]
        elif isinstance(subtypeobj,(str,unicode)):
            st = subtypeobj
        elif isinstance(subtypeobj,dict):
            st = typeFromJson(subtypeobj)
        else:
            raise ValueError("Invalid JSON object specifying Type subtype")
    return Type(byteify(jsonObj['char']),jsonObj.get('size',None),st)

def contextToJson(ctx,saveFunctions=False):
    """Produces a JSON object from a context.  Only the names for userData and customFunctions are saved.
    If saveFunctions=False, customFunctions are not saved"""
    res = {}
    if len(ctx.variables) > 0:
        varjson = []
        for v in ctx.variables:
            varjson.append({'name':v.name,'type':typeToJson(v.type)})
        res['variables'] = varjson
    if len(ctx.expressions) > 0:
        exprjson = {}
        for n,e in ctx.expressions.iteritems():
            exprjson[n] = exprToJson(e)
        res['expressions'] = exprjson
    if saveFunctions and len(ctx.customFunctions) > 0:
        res['customFunctions'] = ctx.customFunctions.keys()
    if len(ctx.userData) > 0:
        res['userData'] = ctx.userData.keys()
    return res

def contextFromJson(context,jsonObj):
    """Creates a context from a JSON object previously saved by context.toJson().  
    userData is not restored and customFunctions are not restored, but rather,
    userData and customFunctions are assumed to have been set up with exactly the same keys
    as when toJson was called.

    Modifies context in-place.
    """
    if 'userData' in jsonObj:
        for d in jsonObj['userData']:
            if d not in context.userData:
                print "Context.fromJson(): Warning, item",d,"is not yet in userData"
    if 'customFunctions' in jsonObj:
        for d in jsonObj['customFunctions']:
            if d not in context.customFunctions:
                print "Context.fromJson(): Warning, item",d,"is not yet in customFunctions"
    context.variables = []
    context.variableDict = dict()
    context.expressions = dict()
    if 'variables' in jsonObj:
        for v in jsonObj['variables']:
            context.addVar(v['name'],typeFromJson(v['type']))
    if 'expressions' in jsonObj:
        for n,v in jsonObj['expressions'].iteritems():
            context.expressions[n] = exprFromJson(context,v)
    return context

def toStr(obj,parseCompatible=True):
    if not isinstance(obj,Expression):
        raise ValueError("Can only convert Expressions to strings")
    return exprToStr(obj,parseCompatible)

def toJson(obj):
    if isinstance(obj,Expression):
        return exprToJson(obj)
    elif isinstance(obj,Context):
        return contextToJson(obj)
    else:
        raise ValueError("Argument needs to be an Expression or Context")


def latex(expr):
    """Returns LaTeX code for the Expression expr.  Requires Sympy."""
    try:
        import sympy
        import symbolic_sympy
    except ImportError as e:
        raise e
        raise RuntimeError("Sympy is required for conversion to latex")
    return sympy.latex(symbolic_sympy.exprToSympy(expr))

def pprint(expr):
    """Pretty-prints the Expression expr.  If Sympy is installed it will use the sympy
    pretty-printer."""
    try:
        import sympy
        import symbolic_sympy
        sympy.pprint(symbolic_sympy.exprToSympy(expr),use_unicode=False)
    except ImportError:
        print exprToStr(expr,parseCompatible=False)
    except TypeError:
        print exprToStr(expr,parseCompatible=False)
    except ValueError:
        print exprToStr(expr,parseCompatible=False)

def codegen(name_expr,language=None,**options):
    """Similar to sympy.codegen. Generates one or more expressions in the target language.
    Requires Sympy.

    - name_expr: a single (name, Expression) tuple or a list of (name, Expression) tuples.
      It is also acceptable to put in Function objects, provided that they are defined via
      Expressions rather than Python functions.  To get multiple return values, you can provide
      a list of (Variable == Expression) expressions.
    - language: any language accepted by Sympy.
    - options: other options for sympy.codegen.

    See http://docs.sympy.org/latest/modules/utilities/codegen.html
    """
    try:
        import sympy
        import symbolic_sympy
        from sympy.utilities.codegen import codegen
    except ImportError:
        raise RuntimeError("Sympy is required for codegen")
    def convert(ne):
        if isinstance(ne,Function):
            if not isinstance(ne.func,Expression):
                raise ValueError("Can't do code generation for plain Python function %s"%(ne.name,))
            sexpr = symbolic_sympy.exprToSympy(ne.expr)
            return (ne.name,sexpr)
        if not isinstance(ne,(list,tuple)) or len(ne)!=2 or not isinstance(ne[0],str):
            raise ValueError("Input must be a (str,Expression) pair.")
        name,expr_or_exprs = ne
        sexpr = None
        if not isinstance(expr_or_exprs,Expression):
            if not isinstance(expr_or_exprs,(list,tuple)):
                raise ValueError("Input must consist of one or more (str,Expression) pairs.")
            sexpr = []
            for expr in expr_or_exprs:
                sexpr.append(symbolic_sympy.exprToSympy(expr))
        else:
            sexpr = symbolic_sympy.exprToSympy(expr_or_exprs)
        return name,sexpr
    if hasattr(name_expr,'__iter__') and isinstance(name_expr[0],(list,tuple)):
        s_name_expr = [convert(x) for x in name_expr]
    else:
        s_name_expr = convert(name_expr)
    return codegen(s_name_expr,language=language,**options)

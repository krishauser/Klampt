from klampt import RobotModel,WorldModel,Geometry3D,GeometricPrimitive,VolumeGrid,PointCloud,Appearance
from klampt.vis import GLProgram,camera,gldraw
import klampt.math.vectorops as op
import klampt.math.se3 as se3
import klampt.math.so3 as so3
import math,os
if os.path.exists(".vapory"):
    import vapory.vapory as vp
else: import vapory as vp
import subprocess

def patch_vapory():
    for name in ['Mesh','Mesh2','VertexVectors','NormalVectors','FaceIndices']:
        if name not in dir(vp):
            setattr(vp,name,type(name,(vp.POVRayElement,object),{}))

def get_property(properties,objects,name,default=None):
    if objects is None:
        return properties[name] if name in properties else default
    elif isinstance(objects,list) and len(objects)==0:
        return properties[name] if name in properties else default
    elif isinstance(objects,tuple) and len(list(objects))==0:
        return properties[name] if name in properties else default
    else:
        for o in objects:
            if o in properties and name in properties[o]:
                return properties[o][name]
            if callable(getattr(o,"getName",None)):
                if o.getName() in properties and name in properties[o.getName()]:
                    return properties[o.getName()][name]
        return properties[name] if name in properties else default
    
def get_property_yes(properties,objects,name,default=None):
    ret=get_property(properties,objects,name,default)
    return ret is not None and (not isinstance(ret,int) or ret>0)

def create_env_light_for_bb(bb,distRatio=1.5,zdim=2,nLight=3):
    xdim=(zdim+1)%3
    ydim=(zdim+2)%3
    
    ctr=[0.,0.,0.]
    ctr[xdim]=(bb[0][xdim]+bb[1][xdim])/2
    ctr[ydim]=(bb[0][ydim]+bb[1][ydim])/2
    ctr[zdim]=(bb[1][zdim]-bb[0][zdim])*distRatio+bb[1][zdim]
    
    x=(bb[1][xdim]-bb[0][xdim])/2
    y=(bb[1][ydim]-bb[0][ydim])/2
    dist=op.norm((x,y))*(1.+distRatio)
    
    pos=[]
    for d in range(nLight):
        ang=math.pi*2*d/nLight
        ctrd=[c for c in ctr]
        ctrd[xdim]+=dist*math.cos(ang)
        ctrd[ydim]+=dist*math.sin(ang)
        pos.append(ctrd)
        
    tgt=[c for c in ctr]
    tgt[zdim]=bb[0][zdim]
    return pos,tgt

def geometry_to_povray(appearance,geometry,object,transform,properties):
    if get_property_yes(properties,[geometry,object],"hide"):
        return []
    
    #analyze appearance
    tex=get_property(properties,[geometry,object],'texture')
    if tex is None:
        tex_params=[]
        #pigment
        pigment=get_property(properties,[geometry,object],'pigment')
        if pigment is None:
            transmit=1.-appearance.getColor()[3]
            pigment=vp.Pigment(*['color',list(appearance.getColor())[0:3],
				 'transmit',get_property(properties,[geometry,object],'ambient',transmit)])
        tex_params.append(pigment)
        
        #finish
        finish=get_property(properties,[geometry,object],'finish')
        if finish is None:
            finish=vp.Finish(*['ambient',get_property(properties,[geometry,object],'ambient',0.2), \
                               'diffuse',get_property(properties,[geometry,object],'diffuse',0.7), \
                               'phong',get_property(properties,[geometry,object],'phong',1.),  \
                               'phong_size',get_property(properties,[geometry,object],'phong_size',50)])
        tex_params.append(finish)
        
        #normal
        normal=get_property(properties,[geometry,object],'normal')
        if normal is not None:
            tex_params.append(normal)
        
        #texture
        tex=vp.Texture(*tex_params)
    
    #create geometry
    ret=[]
    if transform is None:
        transform=geometry.getCurrentTransform()
    else: transform=se3.mul(transform,geometry.getCurrentTransform())
    if geometry.type()=="GeometricPrimitive":
        prim=geometry.getGeometricPrimitive()
        if get_property_yes(properties,[prim,geometry,object],"hide"):
            return ret
        if prim.type=="Point":
            rad=get_property(properties,[prim,geometry,object],"radius")
            if rad is not None:
                mesh_param=[se3.apply(transform,prim.properties[0:3]),rad]
                mesh_param.append(tex)
                mesh=vp.Sphere(*mesh_param)
                ret.append(mesh)
        elif prim.type=="Sphere":
            mesh_param=[se3.apply(transform,prim.properties[0:3]),prim.properties[3]]
            mesh_param.append(tex)
            mesh=vp.Sphere(*mesh_param)
            ret.append(mesh)
        elif prim.type=="Segment":
            rad=get_property(properties,[prim,geometry,object],"radius")
            if rad is not None:
                mesh_param=[se3.apply(transform,prim.properties[0:3]),se3.apply(transform,prim.properties[3:6]),rad]
                mesh_param.append(tex)
                mesh=vp.Cylinder(*mesh_param)
                ret.append(mesh)
        elif prim.type=="AABB":
            mesh_param=[se3.apply(transform,prim.properties[0:3]),se3.apply(transform,prim.properties[3:6])]
            mesh_param.append(tex)
            mesh=vp.Box(*mesh_param)
            ret.append(mesh)
    elif geometry.type()=="Group":
        for idElem in range(geometry.numElements()):
            elem=geometry.getElement(idElem)
            elem.getCurrentTransform()
            ret+=geometry_to_povray(appearance=appearance,geometry=elem,object=object,transform=transform,properties=properties)
    elif geometry.type()=="TriangleMesh":
        tm=geometry.getTriangleMesh()
        
        if get_property_yes(properties,[geometry,object],"smooth"):
            vss=[se3.apply(transform,tuple(tm.vertices[i*3:i*3+3])) for i in range(len(tm.vertices)//3)]
            iss=[tuple(tm.indices[i*3:i*3+3]) for i in range(len(tm.indices)//3)]
            mesh_param=[vp.VertexVectors(*([len(vss)]+vss)),vp.FaceIndices(*([len(iss)]+iss))]
            mesh_param.append(tex)
            mesh=vp.Mesh2(*mesh_param)
        else:
            vss=[se3.apply(transform,tuple(tm.vertices[i*3:i*3+3])) for i in range(len(tm.vertices)//3)]
            iss=[tuple(tm.indices[i*3:i*3+3]) for i in range(len(tm.indices)//3)]
            mesh_param=[vp.Triangle(vss[it[0]],vss[it[1]],vss[it[2]]) for it in iss]
            mesh_param.append(tex)
            mesh=vp.Mesh(*mesh_param)
        
        ret.append(mesh)
    elif geometry.type()=="VolumeGrid":
        from skimage import measure
        import numpy as np
        grid=geometry.getVolumeGrid()
        volume=np.reshape(np.array(list(grid.values)),tuple(grid.dims))
        spacing=[(b-a)/d for a,b,d in zip(grid.bbox[0:3],grid.bbox[3:6],grid.dims[0:3])]
        vss,iss,nss,_=measure.marching_cubes_lewiner(volume,level=0.,spacing=spacing)
        vss+=np.expand_dims(np.array(grid.bbox[0:3]).T,0)
        vss=[vss[it,:].tolist() for it in range(vss.shape[0])]
        iss=[iss[it,:].tolist() for it in range(iss.shape[0])]
        nss=[nss[it,:].tolist() for it in range(nss.shape[0])]
        mesh_param=[vp.VertexVectors(*([len(vss)]+vss)),vp.NormalVectors(*([len(nss)]+nss)),vp.FaceIndices(*([len(iss)]+iss))]
        mesh_param.append(tex)
        mesh=vp.Mesh2(*mesh_param)
        ret.append(mesh)
    elif geometry.type()=="PointCloud":
        cloud_param=[]
        cloud=geometry.getPointCloud()
        rad=get_property(properties,[cloud,geometry,object],"radius")
        for id in range(len(cloud.vertices)//3):
            cloud_param.append(vp.Sphere(cloud.vertices[id*3:id*3+3],rad))
        cloud_param.append(tex)
        mesh=vp.Union(*cloud_param)
        ret.append(mesh)
    else:
        print("Geometry (name=%s) type: %s not supported!"%(object.getName(),geometry.type()))
    return ret

def render_povstring(string, outfile=None, height=None, width=None,
                     quality=None, antialiasing=None, remove_temp=True,
                     show_window=False, tempfile=None, includedirs=None,
                     output_alpha=False, render=True):

    """ Renders the provided scene description with POV-Ray.

    Args:
        string (str): A string representing valid POVRay code. Typically, it
            will be the result of ``scene(*objects)``
        outfile (str, optional): Name of the PNG file for the output. If
            outfile is None, a numpy array is returned (if numpy is installed).
            If outfile is 'ipython' and this function is called last in an 
            IPython notebook cell, this will print the result in the notebook.

        height (int, optional): height in pixels
        width (int, optional): width in pixels
        output_alpha (bool, optional):  If true, the background will be
            transparent, rather than the default black background.  Note that
            this option is ignored if rendering to a numpy array, due to
            limitations of the intermediate ppm format.
    """

    pov_file = tempfile or '__temp__.pov'
    with open(pov_file, 'w+') as f:
        f.write(string)

    return_np_array = (outfile is None)
    display_in_ipython = (outfile=='ipython')

    format_type = "P" if return_np_array else "N"

    if return_np_array:
        outfile='-'

    if display_in_ipython:
        outfile = '__temp_ipython__.png'

    cmd = ["povray.exe" if os.name=='nt' else "povray", pov_file]
    if height is not None: cmd.append('+H%d'%height)
    if width is not None: cmd.append('+W%d'%width)
    if quality is not None: cmd.append('+Q%d'%quality)
    if antialiasing is not None: cmd.append('+A%f'%antialiasing)
    if output_alpha: cmd.append('Output_Alpha=on')
    if not show_window:
        cmd.append('-D')
    else:
        cmd.append('+D')
    if includedirs is not None:
        for dir in includedirs:
            cmd.append('+L%s'%dir)
    cmd.append("Output_File_Type=%s"%format_type)
    cmd.append("+O%s"%outfile)
    if return_np_array:
        return cmd
        
    process = subprocess.Popen(cmd, stderr=subprocess.PIPE,
                                    stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE)

    out, err = process.communicate(string.encode('ascii'))

    if remove_temp:
        os.remove(pov_file)

    if process.returncode:
        print(type(err), err)
        raise IOError("POVRay rendering failed with the following error: "+err.decode('ascii'))

    if return_np_array:
        return ppm_to_numpy(buffer=out)

    if display_in_ipython:
        if not ipython_found:
            raise("The 'ipython' option only works in the IPython Notebook.")
        return Image(outfile)

def to_povray(vis,world,properties={}):
    """Convert a frame in klampt visualization to a povray script (or render
    to an image)

    Args:
        vis: sub-class of GLProgram
        world: sub-class of WorldModel
        properties: some additional information that povray can take but does
            not map to anything in klampt.
    
    Note::
        Do not modify properties, use the convenience functions that I provide 
        below (after this function).  These take the form ``render_to_x`` and
        ``add_x``.
    """
    #patch on vapory
    patch_vapory()
    
    #camera
    mat=vis.view.camera.matrix()
    pos=mat[1]
    right=mat[0][0:3]
    up=mat[0][3:6]
    dir=op.mul(mat[0][6:9],-1)
    tgt=op.add(mat[1],dir)
    #scale
    fovy=vis.view.fov*vis.view.h/vis.view.w
    fovx=math.atan(vis.view.w*math.tan(fovy*math.pi/360.)/vis.view.h)*360./math.pi
    right=op.mul(right,-float(vis.view.w)/vis.view.h)
    #camera
    camera_params=['orthographic' if vis.view.orthogonal else 'perspective',
                   'location',[pos[0],pos[1],pos[2]],
                   'look_at',[tgt[0],tgt[1],tgt[2]],
                   'right',[right[0],right[1],right[2]],
                   'up',[up[0],up[1],up[2]],
                   'angle',fovx,
                   'sky',get_property(properties,[],"sky",[0.,0.,1.])]
    camera=vp.Camera(*camera_params)
    
    #tempfile
    tempfile=get_property(properties,[],"tempfile",None)
    tempfile_path=os.path.dirname(tempfile) if tempfile is not None else '.'
    if not os.path.exists(tempfile_path):
        os.mkdir(tempfile_path)
    
    #objects
    objects=[]
    objs=[o for o in properties["visualObjects"]] if "visualObjects" in properties else []
    objs+=[world.terrain(i) for i in range(world.numTerrains())]
    objs+=[world.rigidObject(i) for i in range(world.numRigidObjects())]
    for r in range(world.numRobots()):
        objs+=[world.robot(r).link(i) for i in range(world.robot(r).numLinks())]
    for obj in objs:
        transient=get_property(properties,[obj],"transient",default=True)
        if transient:
            objects+=geometry_to_povray(obj.appearance(),obj.geometry(),obj,None,properties=properties)
        else: 
            path=tempfile_path+'/'+obj.getName()+'.pov'
            if not os.path.exists(path):
                R,t=obj.geometry().getCurrentTransform()
                obj.geometry().setCurrentTransform([1,0,0,0,1,0,0,0,1],[0,0,0])
                geom=geometry_to_povray(obj.appearance(),obj.geometry(),obj,None,properties=properties)
                if len(geom)>1:
                    file_content=vp.Union(*geom)
                elif len(geom)>0: 
                    file_content=vp.Object(*geom)
                else: file_content=None
                if file_content is not None:
                    f=open(path,'w')
                    f.write(str(file_content))
                    f.close()
                    obj.geometry().setCurrentTransform(R,t)
                else: path=None
            #include    
            if path is not None:
                R,t=obj.geometry().getCurrentTransform()
                objects.append(vp.Object('#include "%s"'%path,"matrix",R+t))
            
    #light
    if "lights" in properties:
        objects+=properties["lights"]
            
    #scene
    gsettings=[]
    scene=vp.Scene(camera=camera,
                   objects=objects,
                   included=get_property(properties,[],"included",[]),
                   global_settings=get_property(properties,[],"global_settings",[]))
    try:
        #this works with later version of vapory
        return  \
        render_povstring(str(scene),   \
                     outfile=get_property(properties,[],"outfile",None),  \
                     width=vis.view.w,height=vis.view.h,    \
                     quality=get_property(properties,[],"quality",None),    \
                     antialiasing=get_property(properties,[],"antialiasing",0.3),    \
                     remove_temp=get_property(properties,[],"remove_temp",False),    \
                     show_window=get_property(properties,[],"show_window",False),     \
                     tempfile=tempfile, \
                     includedirs=get_property(properties,[],"includedirs",None),    \
                     output_alpha=get_property(properties,[],"output_alpha",True))
    except:
        #this works with earlier version of vapory
        return  \
        render_povstring(str(scene),   \
                     outfile=get_property(properties,[],"outfile",None),  \
                     width=vis.view.w,height=vis.view.h,    \
                     quality=get_property(properties,[],"quality",None),    \
                     antialiasing=get_property(properties,[],"antialiasing",0.3),    \
                     remove_temp=get_property(properties,[],"remove_temp",False))
    
def union_bb(a,b):
    if a is None:
        return b
    elif b is None:
        return a
    elif isinstance(b,list):
        return union_bb(a,(b,b))
    else:
        return ([min(c[0],c[1]) for c in zip(a[0],b[0])],   \
                [max(c[0],c[1]) for c in zip(a[1],b[1])])
    
def compute_bb(entity):
    if isinstance(entity,RobotModel):
        bb=None
        for i in range(entity.numLinks()):
            bb=union_bb(compute_bb(entity.link(i)),bb)
        return bb
    return entity.geometry().getBBTight()
    
def render_to_file(properties,file):
    """This will setup your properties dict to call povray to give you an image"""
    properties['tempfile']=None
    properties['remove_temp']=True
    properties['outfile']=file
    
def render_to_animation(properties,folder):
    """This will setup your properties dict to not call povray, but rather just
    generate a render script.
    """
    if not os.path.exists(folder):
        os.mkdir(folder)
        
    import re
    maxId=-1
    for f in os.listdir(folder):
        if re.match('[0-9]+.pov',f): 
            maxId=max(maxId,int(f[:len(f)-4]))
    maxId+=1
            
    properties['tempfile']=folder+"/"+str(maxId)+".pov"
    properties['remove_temp']=False
    properties['outfile']=None
        
def add_light(properties,pos,tgt=None,color=[1.,1.,1.], \
              spotlight=False,radius=15.,falloff=20.,tightness=10., \
              area=0.,sample=9,adaptive=True,jitter=True):
    if 'lights' not in properties:
        properties['lights']=[]
    
    if isinstance(pos,list) and isinstance(pos[0],list):
        if isinstance(tgt,list) and isinstance(tgt[0],list):
            for p,t in zip(pos,tgt):
                add_light(properties,pos=p,tgt=t,color=color,   \
                          spotlight=spotlight,radius=radius,falloff=falloff,tightness=tightness,    \
                          area=area,sample=sample,adaptive=adaptive,jitter=jitter)
        else:
            for p in pos:
                add_light(properties,pos=p,tgt=tgt,color=color, \
                          spotlight=spotlight,radius=radius,falloff=falloff,tightness=tightness,    \
                          area=area,sample=sample,adaptive=adaptive,jitter=jitter)
    else:
        light_params=[list(pos),'color',list(color)]
        if spotlight:
            light_params+=['spotlight','radius',radius,'falloff',falloff,'tightness',tightness,'point_at',tgt]
        if area>0.:
            d=op.sub(tgt,pos)
            d=op.mul(d,1/op.norm(d))
            minId=None
            for i in range(3):
                if abs(d[i])<=abs(d[(i+1)%3]) and abs(d[i])<=abs(d[(i+2)%3]):
                    minId=i
            t0=[0,0,0]
            t0[minId]=1.
            t1=op.cross(d,t0)
            t1=op.mul(t1,1/op.norm(t1))
            t0=op.cross(d,t1)
            light_params+=['area_light',list(op.mul(t0,area)),list(op.mul(t1,area)),sample,sample,'adaptive',1,'jitter']
        properties['lights'].append(vp.LightSource(*light_params))

def add_multiple_lights(properties,object,dist,numLight,gravity=[0,0,-9.81],tgt=None,color=[1.,1.,1.], \
              spotlight=False,radius=15.,falloff=20.,tightness=10., \
              area=0.,sample=9,adaptive=True,jitter=True):
    """This is a convenient function that calls add_light multiple times"""    
    #normalize gravity
    g=op.mul(gravity,-1/op.norm(gravity))
    
    #compute frame
    gabs=[abs(gi) for gi in g]
    id=gabs.index(min(gabs))
    t0=[1. if i==id else 0. for i in range(3)]
    t1=op.cross(t0,g)
    t1=op.mul(t1,1/op.norm(t1))
    t0=op.cross(t1,g)
    
    #find highest direction
    bb=compute_bb(object)
    ctr=op.mul(op.add(bb[0],bb[1]),0.5)
    distg=sum([abs((bb[1][i]-bb[0][i])/2*g[i]) for i in range(3)])
    
    #add each light
    for i in range(numLight):
        angle=math.pi*2*i/numLight
        d0=op.mul(g,distg)
        d1=op.mul(t0,math.sin(angle)*dist)
        d2=op.mul(t1,math.cos(angle)*dist)
        add_light(properties,op.add(d0,op.add(d1,d2)),ctr,color,
                  spotlight,radius,falloff,tightness,area,sample,adaptive,jitter)
        
def mark_transient(properties,object,transient=False):
    """If you know object will not change over time, use this function to tell
    povray.  This saves computation."""
    if object not in properties:
        properties[object.getName()]={}
    properties[object.getName()]["transient"]=transient

def mark_terrain_transient(properties,world,transient=False):
    for i in range(world.numTerrains()):
        mark_transient(properties,world.terrain(i),transient)
        
def mark_robot_transient(properties,robot,transient=False):
    for i in range(robot.numLinks()):
        mark_transient(properties,robot.link(i),transient)

def set_material(properties,object,finish,normal):
    """Override default color-only material with more advanced material specification in povray

    Args:
        finish: a vapory.Finish object
        normal: a vapory.Normal object
    
    setting up these two material parameters is not easy and requires artistic tuning.
    TODO: create a preset library of materials and convenient functions.
    """
    if object not in properties:
        properties[object.getName()]={}
    properties[object.getName()]["finish"]=finish
    properties[object.getName()]["normal"]=normal
    
